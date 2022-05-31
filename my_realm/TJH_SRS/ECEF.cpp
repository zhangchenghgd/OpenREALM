#include "ECEF.h"
#include <Eigen/Eigen>

namespace TJH
{
    namespace SRS
    {
        ///< 初始化默认 WGS84 椭球
        static const GeoEllipsoid GeoEllipsoid_WGS84 = GeoEllipsoid(WGS84_A, WGS84_B);

        const GeoEllipsoid &GeoEllipsoid::WGS84() { return GeoEllipsoid_WGS84; }

        static const ECEF ECEF_WGS84 = ECEF(GeoEllipsoid_WGS84);

        ECEF::ECEF(const GeoEllipsoid &ellipsoid)
            : Ellipsoid(ellipsoid)
        {
        }

        const ECEF &ECEF::WGS84()
        {
            return ECEF_WGS84;
        }

        Coord3 ECEF::fromLonLatAlt(const LonLatAlt &lla_coord) const
        {
            const double a = Ellipsoid.a();
            const double b = Ellipsoid.b();
            const double a2 = pow(a, 2);
            const double b2 = pow(b, 2);
            const double e2 = (a2 - b2) / a2;
            
            double lon = lla_coord.x() * M_PI / 180.0;
            double lat = lla_coord.y() * M_PI / 180.0;
            double alt = lla_coord.z();

            const double clat = cos(lat);
            const double slat = sin(lat);
            const double clon = cos(lon);
            const double slon = sin(lon);

            const double N = a / sqrt(1.0 - e2 * pow(slat,2));

            const double x = (N + alt) * clat * clon;
            const double y = (N + alt) * clat * slon;
            const double z = (N * (1.0 - e2) + alt) * slat;

            return Coord3(x, y, z);
        }

        LonLatAlt ECEF::toLonLatAlt(const Coord3 &ecef_coord) const
        {
            const double a = Ellipsoid.a();
            const double b = Ellipsoid.b();
            const double e = Ellipsoid.e();   // 第一偏心率
            const double ep = Ellipsoid.ep(); // 第二偏心率
            const double a2 = pow(a, 2);
            const double b2 = pow(b, 2);
            const double e2 = (a2 - b2) / a2;
            const double ep2 = (a2 - b2) / b2;

            const double x = ecef_coord.x();
            const double y = ecef_coord.y();
            const double z = ecef_coord.z();

            const double p = hypot(x, y);
            const double th = atan2(a * z, b * p);
            const double lon = atan2(y, x);
            const double lat = atan2((z + ep2 * b * pow(sin(th), 3)),
                                     (p - e2 * a * pow(cos(th), 3)));

            const double slat = sin(lat);

            // 曲率半径
            const double N = a / sqrt(1 - e2 * pow(slat, 2));
            const double alt = p / cos(lat) - N;

            return Coord3(lon * 180.0 / M_PI,
                          lat * 180.0 / M_PI,
                          alt);
        }

    } // namespace SRS
} // namespace TJH