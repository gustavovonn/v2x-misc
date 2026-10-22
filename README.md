# v2x-misc

Reusable V2X motion primitives. Header-only and free of any detection policy, so it
cross-compiles for embedded targets and can be shared by experiments that use a different
confidence mechanism. Standard library only.

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

Usage: add `libs/v2x-misc/include` to the include path.