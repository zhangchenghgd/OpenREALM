/**
 * @file ECEF.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 地心地固坐标系定义
 * @version 0.1
 * @date 2020-12-11
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_ECEF_H_
#define TJH_SRS_ECEF_H_

#include "SrsExport.h"
#include "Coord.h"
#include <math.h>

namespace TJH
{
    namespace SRS
    {
        // WGS84 椭球常量
        const double WGS84_A = 6378137.0;            ///< WGS84 椭球长半轴
        const double WGS84_B = 6356752.3142451793;   ///< WGS84 椭球短半轴

        /**
         * @brief 地球椭球体
         */
        struct TJH_SRS_EXPORT GeoEllipsoid
        {
            double Semimajor; ///< 椭球长半轴
            double Semiminor; ///< 椭球短半轴

            /**
             * @brief 构造 GeoEllipsoid 对象
             * @param major 椭球长半轴
             * @param minor 椭球短半轴
             */
            inline GeoEllipsoid(double major, double minor) : Semimajor(major),
                                                              Semiminor(minor)
            {
            }

            /**
             * @brief 长半轴
             */
            inline double a() const { return Semimajor; }

            /**
             * @brief 短半轴
             */
            inline double b() const { return Semiminor; }

            /**
             * @brief 扁率
             * @return double 
             */
            inline double f() const { return (Semimajor - Semiminor) / Semimajor; }

            /**
             * @brief 第一偏心率
             */
            inline double e() const
            {
                return sqrt((pow(Semimajor, 2) - pow(Semiminor, 2)) / pow(Semimajor, 2));
            }

            /**
             * @brief 第二偏心率
             */
            inline double ep() const
            {
                return sqrt((pow(Semimajor, 2) - pow(Semiminor, 2)) / pow(Semiminor, 2));
            }

            /**
             * @brief WGS84 椭球体
             */
            static const GeoEllipsoid &WGS84();
        };

        /**
         * @brief ECEF 地心地固坐标系
         */
        class TJH_SRS_EXPORT ECEF
        {
        public:
            GeoEllipsoid Ellipsoid;   ///<  椭球体

        public:
            /**
             * @brief 构造 ECEF 对象，默认采用WGS84椭球
             * @param semiMajor 
             * @param semiMinor 
             */
            ECEF(const GeoEllipsoid &ellipsoid = GeoEllipsoid::WGS84());

            /**
             * @brief 坐标转换：经纬度 => ECEF
             * @param[in] lla_coord 经纬度坐标，采用ECEF椭球
             * @return ECEF坐标 
             */
            Coord3 fromLonLatAlt(const LonLatAlt &lla_coord) const;

            /**
             * @brief 坐标转换：ECEF => 经纬度
             * @param[in] ecef_coord ECEF坐标
             * @return 经纬度坐标，采用ECEF椭球
             */
            LonLatAlt toLonLatAlt(const Coord3 &ecef_coord) const;

            /**
             * @brief 地心地固坐标系(EPSG:4978)，使用WGS84椭球
             * @return ECEF 
             */
            static const ECEF &WGS84();
        };

    } // namespace SRS
} // namespace TJH

#endif //  !TJH_SRS_ECEF_H_
