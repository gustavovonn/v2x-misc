# v2x-misc

Reusable V2X motion primitives. Header-only and free of any detection policy, so it
cross-compiles for embedded targets and can be shared by experiments that use a different
confidence mechanism. Standard library only, except the two filtering headers, which need
[Eigen](https://gitlab.com/libeigen/eigen) (3.4) on the include path.

| Header | Contents |
|---|---|
| `Constants.h` | WGS-84 ellipsoid parameters and radii of curvature; numerical guards of the motion math |
| `MotionTypes.h` | `KinematicState`, `PredictedState` |
| `Angles.h` | Angle wrapping; compass (clockwise from north) <-> Cartesian (counter-clockwise from east) |
| `CoordinatePolicies.h` | Cartesian and WGS-84 policies: `translate`, `distance`, `displacement`, `bearing` |
| `LocalTangentPlane.h` | WGS-84 to local East-North metres around a fixed origin, and back |
| `TransportTheorem.h` | Position and velocity of a point seen from a moving, rotating frame (`v = v_O + v_rel + w x r`) |
| `CtraKinematics.h` | Coordinate-free CTRA integrator with constant-acceleration fallback |
| `KinematicRegression.h` | OLS estimation of acceleration and yaw rate over a history window |
| `MotionModels.h` | `IMotionModel`, single-step and history-aware CTRA models, `MotionModelFactory` |
| `ConsistencyCheck.h` | Reported vs implied speed and frozen position between two reports of one source; thresholds supplied by the caller |
| `UnscentedKalmanFilter.h` | UKF with augmented process noise and additive measurement noise; split into `predict`, `innovate` and `update` so a measurement can be gated before it is fused (Eigen) |
| `CtraProcessModel.h` | CTRA state `[x, y, heading, v, a, w]` and its process function for the UKF, built on `integrateCtra` (Eigen) |

Usage: add `libs/v2x-misc/include` to the include path (and Eigen for the filtering headers).
There is nothing to compile or link.

## References

- E. A. Wan, R. van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation", IEEE AS-SPCC 2000.
- S. J. Julier, J. K. Uhlmann, "Unscented Filtering and Nonlinear Estimation", Proc. IEEE 92(3), 2004.
