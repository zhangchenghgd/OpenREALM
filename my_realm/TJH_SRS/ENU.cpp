/**
 * @file ENU.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ENU.h"
#include "ECEF.h"
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

namespace TJH
{
    namespace SRS
    {

        

        ENU::ENU(const GeoEllipsoid &ellipsoid) : m_ecef(ECEF(ellipsoid)),
                                                  m_origin(LonLatAlt())
        {
        }

        ENU::ENU(const LonLatAlt &_origin, const GeoEllipsoid &ellipsoid)
        {
            m_ecef = ECEF(ellipsoid);
            setOrigin(_origin);
        }

        ENU::ENU(double longitude, double latitude, double altitude,
                 const GeoEllipsoid &ellipsoid)
        {
            m_ecef = ECEF(ellipsoid);
            setOrigin(LonLatAlt(longitude, latitude, altitude));
        }

        ENU::~ENU()
        {
        }

        void ENU::setOrigin(const LonLatAlt &_origin)
        {
            // 断言：经度范围[-180.0, 180.0]，纬度范围[-90.0, 90.0]
            assert(_origin.x() >= -180.0 && _origin.x() <= 180.0);
            assert(_origin.y() >= -90.0 && _origin.y() <= 90.0);

            m_origin = _origin;
            // 获取在ECEF坐标系下的原点坐标
            Coord3 ecef_o = ecef_origin();

            // 计算旋转矩阵
            double o_lon = m_origin.x();
            double o_lat = m_origin.y();
            double lbd = o_lon * M_PI / 180.0;
            double phi = o_lat * M_PI / 180.0;
            Eigen::Matrix3d mat;
            mat(0, 0) = -sin(lbd);
            mat(0, 1) = cos(lbd);
            mat(0, 2) = 0.0;
            mat(1, 0) = -sin(phi) * cos(lbd);
            mat(1, 1) = -sin(phi) * sin(lbd);
            mat(1, 2) = cos(phi);
            mat(2, 0) = cos(phi) * cos(lbd);
            mat(2, 1) = cos(phi) * sin(lbd);
            mat(2, 2) = sin(phi);
            Eigen::Matrix3d mat_tp = mat.transpose();

            // 计算 ENU => ECEF 下的TRS变换
            m_ENU_TO_ECEF_RTS.S = 1.0;
            m_ENU_TO_ECEF_RTS.T[0] = ecef_o.x();
            m_ENU_TO_ECEF_RTS.T[1] = ecef_o.y();
            m_ENU_TO_ECEF_RTS.T[2] = ecef_o.z();
            m_ENU_TO_ECEF_RTS.R[0] = mat_tp(0, 0);
            m_ENU_TO_ECEF_RTS.R[1] = mat_tp(0, 1);
            m_ENU_TO_ECEF_RTS.R[2] = mat_tp(0, 2);
            m_ENU_TO_ECEF_RTS.R[3] = mat_tp(1, 0);
            m_ENU_TO_ECEF_RTS.R[4] = mat_tp(1, 1);
            m_ENU_TO_ECEF_RTS.R[5] = mat_tp(1, 2);
            m_ENU_TO_ECEF_RTS.R[6] = mat_tp(2, 0);
            m_ENU_TO_ECEF_RTS.R[7] = mat_tp(2, 1);
            m_ENU_TO_ECEF_RTS.R[8] = mat_tp(2, 2);
        }

        const LonLatAlt &ENU::origin() const
        {
            return m_origin;
        }

        void ENU::setEllipsoid(const GeoEllipsoid &ellipsoid)
        {
            m_ecef = ECEF(ellipsoid);
            setOrigin(m_origin);
        }

        const GeoEllipsoid &ENU::getEllipsoid() const
        {
            return m_ecef.Ellipsoid;
        }

        Coord3 ENU::ecef_origin() const
        {
            if( !m_origin.empty())
            {
                return m_ecef.fromLonLatAlt(m_origin);
            }
            else
            {
                return Coord3();
            }
        }

        const RTSTransform &ENU::ecefTransform() const
        {
            return m_ENU_TO_ECEF_RTS;
        }

        LonLatAlt ENU::coordToLonLatAlt(const Coord3 &enu_coord) const
        {
            assert(valid());
            // enu => ecef => lla
            Coord3 ecef_coord = m_ENU_TO_ECEF_RTS(enu_coord);
            LonLatAlt lla_coord = m_ecef.toLonLatAlt(ecef_coord);
            return lla_coord;
        }

        Coord3 ENU::coordToECEF(const Coord3 &enu_coord) const
        {
            assert(valid());
            Coord3 ecef_coord = m_ENU_TO_ECEF_RTS(enu_coord);
            return ecef_coord;
        }

        Coord3 ENU::coordFromLonLatAlt(const LonLatAlt &lla_coord) const
        {
            assert(valid());
            // lla => ecef => enu
            RTSTransform rts_inv = m_ENU_TO_ECEF_RTS.inverse();
            Coord3 ecef_coord = m_ecef.fromLonLatAlt(lla_coord);
            Coord3 enu_coord = rts_inv(ecef_coord);
            return enu_coord;
        }

        Coord3 ENU::coordFromECEF(const Coord3 &ecef_coord) const
        {
            assert(valid());
            RTSTransform rts_inv = m_ENU_TO_ECEF_RTS.inverse();
            return rts_inv(ecef_coord);
        }

        bool ENU::valid() const
        {
            return m_origin.x() >= -180.0 &&
                   m_origin.x() <= 180.0 &&
                   m_origin.y() >= -90.0 &&
                   m_origin.y() <= 90.0;
        }

    } // namespace SRS
} // namespace TJH