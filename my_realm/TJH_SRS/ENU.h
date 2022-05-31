/**
 * @file ENU.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief ENU坐标系定义
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_ENU_H_
#define TJH_SRS_ENU_H_

#include "SrsExport.h"
#include "Coord.h"
#include "ECEF.h"
#include "RTSTransform.h"
#include <string>

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief ENU 站心坐标系，默认采用WGS84椭球
         * @details 定义：以站心（如GPS接收天线中心）为坐标系原点O，Z轴与椭球法线重合，向上为正（天向），\n
         *          y与椭球短半轴重合（北向），x轴与地球椭球的长半轴重合（东向）所构成的直角坐标系，\n
         *          称为当地东北天坐标系（ENU）。
         */
        class TJH_SRS_EXPORT ENU
        {
        public:
            /**
             * @brief 构造 ENU 对象，默认采用WGS84椭球
             */
            ENU(const GeoEllipsoid &ellipsoid = GeoEllipsoid::WGS84());

            /**
             * @brief 构造 ENU 对象
             * @param[in] _origin 坐标系原点，经纬度坐标
             * @param[in] ellipsoid 椭球模型，默认WGS84椭球
             */
            ENU(const LonLatAlt &_origin,
                const GeoEllipsoid &ellipsoid = GeoEllipsoid::WGS84());

            /**
             * @brief 构造 ENU 对象
             * @param[in] longitude 坐标系原点经度
             * @param[in] latitude 坐标系原点纬度
             * @param[in] altitude 坐标系原点海拔
             * @param[in] ellipsoid 椭球模型，默认WGS84椭球
             */
            ENU(double longitude, double latitude, double altitude = 0,
                const GeoEllipsoid &ellipsoid = GeoEllipsoid::WGS84());

            /**
             * @brief 析构 ENU 对象
             */
            ~ENU();

            /**
             * @brief  设置坐标原点
             * @param[in] _origin 坐标系原点，经纬度坐标
             */
            void setOrigin(const LonLatAlt &_origin);

            /**
             * @brief 获取坐标原点
             * @return 坐标原点，WGS84经纬度
             */
            const LonLatAlt &origin() const;

            /**
             * @brief 设置ENU所属的地理坐标系椭球参数
             * @param[in] ellipsoid 地球椭球体
             */
            void setEllipsoid(const GeoEllipsoid &ellipsoid);

            /**
             * @brief 获取ENU所属的地理坐标系椭球参数
             * @return GeoEllipsoid  地球椭球体
             */
            const GeoEllipsoid &getEllipsoid() const;

            /**
             * @brief 获取ECEF坐标系下的原点坐标
             * @return 坐标原点，ECEF坐标系
             */
            Coord3 ecef_origin() const;

            /**
             * @brief 获取ENU转换为ECEF的RTS转换
             * @return RTS转换
             */
            const RTSTransform &ecefTransform() const;

            /**
             * @brief 坐标转换: ENU => 经纬度
             * @param[in] enu_coord ENU坐标系下的点坐标
             * @return 转换后的坐标，经纬度 
             */
            LonLatAlt coordToLonLatAlt(const Coord3 &enu_coord) const;

            /**
             * @brief 坐标转换: ENU => ECEF
             * @param[in] enu_coord ENU坐标系下的点坐标
             * @return 转换后的坐标，ECEF坐标系
             */
            Coord3 coordToECEF(const Coord3 &enu_coord) const;

            /**
             * @brief 坐标转换: WGS84 => ENU
             * @param[in] lla_coord  坐标系点坐标
             * @return 转换后的坐标，ENU坐标系
             */
            Coord3 coordFromLonLatAlt(const LonLatAlt &lla_coord) const;

            /**
             * @brief 坐标转换: ECEF => ENU
             * @param[in] ecef_coord ECEF坐标系点坐标
             * @return 转换后的坐标，ENU坐标系
             */
            Coord3 coordFromECEF(const Coord3 &ecef_coord) const;

            /**
             * @brief 是否合法
             */
            bool valid() const;

        private:
            /**
             * @brief 坐标系原点，经纬度标识(WGS84 EPSG:4326)
             */
            LonLatAlt m_origin;

            /**
             * @brief 所属ECEF坐标系
             */
            ECEF m_ecef;

            /**
             * @brief ENU=>ECEF的RTS转换
             */
            RTSTransform m_ENU_TO_ECEF_RTS;
        };

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_ENU_H_
