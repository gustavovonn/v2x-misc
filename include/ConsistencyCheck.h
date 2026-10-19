#ifndef CONSISTENCY_CHECK_H_
#define CONSISTENCY_CHECK_H_

// Physical invariance between two consecutive reports of the same source: reported speed against
// the speed implied by the displacement, and frozen position.
// From CP-Artery include/Checks.h (consistency check, paper 1 Sec. IV-B-1); thresholds are the
// caller's policy.

#include "MotionTypes.h"

#include <algorithm>
#include <cmath>

struct ConsistencyThresholds {
    double speedTolerance = 0.0;      // |v_avg - v_implied| bound [m/s]
    double minInterFrame = 0.0;       // below: duplicate, no verdict [s]
    double maxInterFrame = 0.0;       // above: stale, no verdict [s]
    double frozenSpeed = 0.0;         // reported speed meaning "moving" [m/s]
    double frozenDisplacement = 0.0;  // displacement below which a moving source is frozen [m]
};

struct ConsistencyResult {
    bool evaluated = false;  // dt inside [minInterFrame, maxInterFrame]
    bool failed = false;
    double excessRatio = 0.0;
    double impliedSpeed = 0.0;
    double dt = 0.0;
};

template <typename CoordPolicy>
inline ConsistencyResult consistencyCheck(const KinematicState& past, const KinematicState& current,
                                          const ConsistencyThresholds& th)
{
    ConsistencyResult res;
    // tolerance for floating-point timestamps (0.1 s steps)
    constexpr double kTimeEps = 1e-6;
    res.dt = current.time - past.time;
    if (res.dt < th.minInterFrame - kTimeEps || res.dt > th.maxInterFrame + kTimeEps) {
        return res;
    }
    res.evaluated = true;

    const double displacement = CoordPolicy::displacement(past.lat, past.lon, current.lat, current.lon);
    res.impliedSpeed = displacement / res.dt;

    const double avgSpeed = 0.5 * (current.speed + past.speed);
    const double discrepancy = std::abs(avgSpeed - res.impliedSpeed);
    const bool speedFalsified = discrepancy > th.speedTolerance;
    const bool frozen = avgSpeed > th.frozenSpeed && displacement < th.frozenDisplacement;

    if (speedFalsified || frozen) {
        res.failed = true;
        res.excessRatio = speedFalsified ? (discrepancy - th.speedTolerance) / th.speedTolerance : 1.0;
    }
    return res;
}

#endif  // CONSISTENCY_CHECK_H_
