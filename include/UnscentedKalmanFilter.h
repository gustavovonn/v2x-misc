#ifndef UNSCENTED_KALMAN_FILTER_H_
#define UNSCENTED_KALMAN_FILTER_H_

// Unscented Kalman filter: augmented process noise, additive measurement noise.
//   UKF equations:        E. A. Wan, R. van der Merwe, "The Unscented Kalman Filter for Nonlinear
//                         Estimation", IEEE AS-SPCC 2000, Algorithm 3.1
//   sigma points/weights: S. J. Julier, J. K. Uhlmann, "Unscented Filtering and Nonlinear
//                         Estimation", Proc. IEEE 92(3), 2004, Sec. III
//   noise handling:       R. Schubert, E. Richter, G. Wanielik, "Comparison and Evaluation of
//                         Advanced Motion Models for Vehicle Tracking", FUSION 2008, Sec. III-2
// Adapted from lisha-artery scenarios/monitorAV/ukf.
// Requires Eigen on the include path.

#include "Angles.h"

#include <Eigen/Dense>

#include <cmath>
#include <functional>

namespace ukf {

using Angles::wrapPi;

// Scaled UT parameters (Wan & van der Merwe, Eq. 15): lambda = alpha^2 * (L + kappa) - L.
// Default alpha = 1, beta = 0, kappa = 3 - L gives lambda = 3 - L, the symmetric set whose
// fourth-order moments match a Gaussian (Julier & Uhlmann, Sec. III).
struct SigmaParams {
    double alpha = 1.0;
    double beta = 0.0;
    double kappa = 0.0;
    bool kappaFromDimension = true;  // kappa = 3 - L
};

// Prior (x, P) moved to a new time; sigma points are kept for the update step
struct Prediction {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd sigma;  // n_x x (2L + 1)
};

// One measurement compared with a prediction
struct Innovation {
    Eigen::VectorXd z;
    Eigen::VectorXd zPred;
    Eigen::VectorXd residual;  // z - zPred, angle wrapped
    Eigen::MatrixXd S;         // innovation covariance
    Eigen::MatrixXd Zsig;      // sigma points in measurement space
    int angle = -1;            // index of an angle in z, -1 if none
};

class UnscentedKalmanFilter
{
public:
    // f: augmented sigma point [x; nu] and dt -> propagated state
    using ProcessModel = std::function<Eigen::VectorXd(const Eigen::VectorXd& xAug, double dt)>;
    // h: state -> measurement
    using MeasurementModel = std::function<Eigen::VectorXd(const Eigen::VectorXd& x)>;

    // nx: state size, nv: process noise size, stateAngle: index of an angle in x, -1 if none
    UnscentedKalmanFilter(int nx, int nv, ProcessModel f, int stateAngle = -1, SigmaParams params = {})
        : m_nx(nx), m_nv(nv), m_f(std::move(f)), m_stateAngle(stateAngle)
    {
        const int L = nx + nv;
        const double kappa = params.kappaFromDimension ? 3.0 - L : params.kappa;
        m_lambda = params.alpha * params.alpha * (L + kappa) - L;

        // Wan & van der Merwe, Eq. 15
        const int n = 2 * L + 1;
        m_wm = Eigen::VectorXd::Constant(n, 0.5 / (L + m_lambda));
        m_wc = m_wm;
        m_wm(0) = m_lambda / (L + m_lambda);
        m_wc(0) = m_wm(0) + (1.0 - params.alpha * params.alpha + params.beta);
    }

    int stateSize() const { return m_nx; }
    int noiseSize() const { return m_nv; }

    // Time update; Q: process noise covariance (nv x nv). False when P is not positive definite.
    bool predict(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                 double dt, Prediction& out) const
    {
        const int L = m_nx + m_nv;

        Eigen::VectorXd xAug = Eigen::VectorXd::Zero(L);
        xAug.head(m_nx) = x;

        Eigen::MatrixXd PAug = Eigen::MatrixXd::Zero(L, L);
        PAug.topLeftCorner(m_nx, m_nx) = P;
        PAug.bottomRightCorner(m_nv, m_nv) = Q;

        // Cholesky square root (Julier & Uhlmann, footnote 6)
        Eigen::LLT<Eigen::MatrixXd> llt(PAug);
        if (llt.info() != Eigen::Success) {
            return false;
        }
        const Eigen::MatrixXd S = std::sqrt(L + m_lambda) * Eigen::MatrixXd(llt.matrixL());

        out.sigma.resize(m_nx, 2 * L + 1);
        out.sigma.col(0) = m_f(xAug, dt);
        for (int i = 0; i < L; ++i) {
            out.sigma.col(i + 1) = m_f(xAug + S.col(i), dt);
            out.sigma.col(i + 1 + L) = m_f(xAug - S.col(i), dt);
        }

        out.x = out.sigma * m_wm;
        out.P = Eigen::MatrixXd::Zero(m_nx, m_nx);
        for (int i = 0; i < out.sigma.cols(); ++i) {
            const Eigen::VectorXd d = stateDiff(out.sigma.col(i), out.x);
            out.P += m_wc(i) * d * d.transpose();
        }
        if (m_stateAngle >= 0) {
            out.x(m_stateAngle) = wrapPi(out.x(m_stateAngle));
        }
        return true;
    }

    // Measurement prediction for z; R: measurement noise covariance; angle: index of an angle in z
    Innovation innovate(const Prediction& pred, const MeasurementModel& h, const Eigen::VectorXd& z,
                        const Eigen::MatrixXd& R, int angle = -1) const
    {
        Innovation in;
        in.z = z;
        in.angle = angle;

        const int n = static_cast<int>(pred.sigma.cols());
        in.Zsig.resize(z.size(), n);
        for (int i = 0; i < n; ++i) {
            in.Zsig.col(i) = h(pred.sigma.col(i));
        }

        in.zPred = in.Zsig * m_wm;
        in.S = R;
        for (int i = 0; i < n; ++i) {
            const Eigen::VectorXd d = measDiff(in.Zsig.col(i), in.zPred, angle);
            in.S += m_wc(i) * d * d.transpose();
        }
        in.residual = measDiff(z, in.zPred, angle);
        return in;
    }

    // Measurement update of the prediction with its innovation
    void update(const Prediction& pred, const Innovation& in, Eigen::VectorXd& x, Eigen::MatrixXd& P) const
    {
        Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(m_nx, in.z.size());
        for (int i = 0; i < pred.sigma.cols(); ++i) {
            Tc += m_wc(i) * stateDiff(pred.sigma.col(i), pred.x)
                          * measDiff(in.Zsig.col(i), in.zPred, in.angle).transpose();
        }
        const Eigen::MatrixXd K = Tc * in.S.inverse();
        x = pred.x + K * in.residual;
        P = pred.P - K * in.S * K.transpose();
        if (m_stateAngle >= 0) {
            x(m_stateAngle) = wrapPi(x(m_stateAngle));
        }
    }

private:
    Eigen::VectorXd stateDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const
    {
        Eigen::VectorXd d = a - b;
        if (m_stateAngle >= 0) {
            d(m_stateAngle) = wrapPi(d(m_stateAngle));
        }
        return d;
    }

    static Eigen::VectorXd measDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b, int angle)
    {
        Eigen::VectorXd d = a - b;
        if (angle >= 0) {
            d(angle) = wrapPi(d(angle));
        }
        return d;
    }

    int m_nx;
    int m_nv;
    ProcessModel m_f;
    int m_stateAngle;
    double m_lambda = 0.0;
    Eigen::VectorXd m_wm;  // mean weights
    Eigen::VectorXd m_wc;  // covariance weights
};

}  // namespace ukf

#endif  // UNSCENTED_KALMAN_FILTER_H_
