#ifndef TRANSPORT_THEOREM_H_
#define TRANSPORT_THEOREM_H_

// Planar transport theorem: state of a point P seen from a frame F that moves with the reference
// point O (position p_O, velocity v_O) and rotates at omega about Up:
//   p_P = p_O + r
//   v_P = v_O + v_rel + omega x r,   omega x r = omega * (-r_north, r_east)
// Vectors in East-North [m, m/s]; omega counter-clockwise (right-hand about Up) [rad/s].

struct PlanarVector {
    double east = 0.0;
    double north = 0.0;
};

// omega x r for omega along Up
inline PlanarVector crossUp(double omega, const PlanarVector& r)
{
    return {-omega * r.north, omega * r.east};
}

inline PlanarVector transportPosition(const PlanarVector& pO, const PlanarVector& r)
{
    return {pO.east + r.east, pO.north + r.north};
}

inline PlanarVector transportVelocity(const PlanarVector& vO, const PlanarVector& vRel, double omega,
                                      const PlanarVector& r)
{
    const PlanarVector w = crossUp(omega, r);
    return {vO.east + vRel.east + w.east, vO.north + vRel.north + w.north};
}

#endif  // TRANSPORT_THEOREM_H_
