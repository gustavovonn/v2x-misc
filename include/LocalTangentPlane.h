#ifndef LOCAL_TANGENT_PLANE_H_
#define LOCAL_TANGENT_PLANE_H_

// WGS-84 <-> local East-North plane around a fixed origin.
// Inverse of Wgs84CoordinatePolicy::translate, with the radii of curvature taken at the origin.

#include "Constants.h"

#include <cmath>

class LocalTangentPlane
{
public:
    LocalTangentPlane(double originLatDeg, double originLonDeg)
        : m_lat0(originLatDeg), m_lon0(originLonDeg)
    {
        const double latRad = originLatDeg * ConstantsWGS84::DEG_TO_RAD;
        m_rPhi = ConstantsWGS84::meridionalRadius(latRad);
        m_rLambdaCos = ConstantsWGS84::primeVerticalRadius(latRad) * std::cos(latRad);
    }

    // degrees -> metres east and north of the origin
    void toLocal(double latDeg, double lonDeg, double& east, double& north) const
    {
        east = (lonDeg - m_lon0) * ConstantsWGS84::DEG_TO_RAD * m_rLambdaCos;
        north = (latDeg - m_lat0) * ConstantsWGS84::DEG_TO_RAD * m_rPhi;
    }

    // metres east and north of the origin -> degrees
    void toGeodetic(double east, double north, double& latDeg, double& lonDeg) const
    {
        latDeg = m_lat0 + north / m_rPhi * ConstantsWGS84::RAD_TO_DEG;
        lonDeg = m_lon0 + east / m_rLambdaCos * ConstantsWGS84::RAD_TO_DEG;
    }

private:
    double m_lat0;
    double m_lon0;
    double m_rPhi;        // meridional radius [m]
    double m_rLambdaCos;  // prime-vertical radius * cos(lat0) [m]
};

#endif  // LOCAL_TANGENT_PLANE_H_
