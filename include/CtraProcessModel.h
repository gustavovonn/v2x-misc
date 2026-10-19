#ifndef CTRA_PROCESS_MODEL_H_
#define CTRA_PROCESS_MODEL_H_

// CTRA process model for UnscentedKalmanFilter.
//   state space and transition: R. Schubert, E. Richter, G. Wanielik, "Comparison and Evaluation of
//   Advanced Motion Models for Vehicle Tracking", FUSION 2008, Eq. 6-9 (integrateCtra)
//   noise: white jerk and yaw acceleration held over the step, the CTRV scheme of
//   lisha-artery scenarios/monitorAV/ukf raised by one derivative
// Requires Eigen on the include path.

#include "Constants.h"
#include "CtraKinematics.h"
#include "MotionTypes.h"

#include <Eigen/Dense>

#include <cmath>

namespace CtraState {

// Local tangent plane: x east, y north [m]; heading and yaw rate clockwise from north, as integrateCtra
enum Index : int {
    X = 0,         // [m]
    Y = 1,         // [m]
    HEADING = 2,   // [rad]
    SPEED = 3,     // [m/s]
    ACCEL = 4,     // [m/s^2]
    YAW_RATE = 5,  // [rad/s]
    SIZE = 6
};

// Augmented noise, appended after the state
enum Noise : int {
    JERK = 0,       // [m/s^3]
    YAW_ACCEL = 1,  // [rad/s^2]
    NOISE_SIZE = 2
};

}  // namespace CtraState

// f(x_aug, dt) for UnscentedKalmanFilter::ProcessModel
inline Eigen::VectorXd ctraProcess(const Eigen::VectorXd& xAug, double dt)
{
    using namespace CtraState;

    const double heading = xAug(HEADING);
    const double a = xAug(ACCEL);
    const double w = xAug(YAW_RATE);
    const double nuJerk = xAug(SIZE + JERK);
    const double nuYawAccel = xAug(SIZE + YAW_ACCEL);

    KinematicState start;
    start.speed = xAug(SPEED);
    start.heading = heading * ConstantsWGS84::RAD_TO_DEG;
    const LocalDisplacement step = integrateCtra(start, a, w, dt);

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    Eigen::VectorXd x(SIZE);
    // heading kept unwrapped so sigma points stay contiguous; the filter wraps the mean
    x(X) = xAug(X) + step.dx + nuJerk * dt3 / 6.0 * std::sin(heading);
    x(Y) = xAug(Y) + step.dy + nuJerk * dt3 / 6.0 * std::cos(heading);
    x(HEADING) = heading + w * dt + 0.5 * nuYawAccel * dt2;
    x(SPEED) = step.speed + 0.5 * nuJerk * dt2;
    x(ACCEL) = a + nuJerk * dt;
    x(YAW_RATE) = w + nuYawAccel * dt;
    return x;
}

#endif  // CTRA_PROCESS_MODEL_H_
