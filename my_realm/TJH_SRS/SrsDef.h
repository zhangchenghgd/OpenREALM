/**
 * @file SrsDef.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 坐标系定义
 * @version 0.1
 * @date 2020-12-13
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_SRSDEF_H_
#define TJH_SRS_SRSDEF_H_

#include "SrsExport.h"
#include "Coord.h"
#include <string>
#include <vector>
#include <map>

namespace TJH
{
    namespace SRS
    {

#pragma region COMMON_SRS_EPSG

        /**
         * @brief [ WGS 1984 ] WGS1984坐标系，EPSG:4326
         */
        const int WGS_84_EPSG = 4326;

        /**
         * @brief [ ECEF ] 地心地固坐标系，WGS84椭球，坐标系EPSG:4978
         */
        const int ECEF_WGS84_EPSG = 4978;

        /**
         * @brief [ Beijing 1954 ] 北京1954大地坐标系，EPSG:4214
         */
        const int Beijing_1954_EPSG = 4214;

        /**
         * @brief [ Xian 1980 ] 西安1980大地坐标系，EPSG:4610
         */
        const int Xian_1980_EPSG = 4610;

        /**
         * @brief [ CGCS2000 ] 2000国家大地坐标系，EPSG:4490
         */
        const int CGCS_2000_EPSG = 4490;

#pragma endregion COMMON_SRS_EPSG

#pragma region LINEARUNITS_REGION
        /**
         * @brief 线性单位
         */
        enum LinearUnits
        {
            arbitrary = 0, ///< 单位：任意
            meter = 1,     ///< 单位：米（m）
            foot = 2,      ///< 单位：英尺（ft）, 换算：1 ft = 0.3048 m
            us_foot = 3,   ///< 单位：英尺（ft）, 换算：1 us_ft = 0.3048006096012192 m
        };

        /**
         * @brief 线性单位字符串
         * @param unit 线性单位
         * @return std::string  {arbitrary, ""}, {meter, "Meter"}, {foot, "Foot"}, {us_foot, "Foot_US"}
         */
        TJH_SRS_EXPORT std::string LinearUnitString(LinearUnits unit);

        /**
         * @brief 线性单位转换为meter
         * @param unit 线性单位
         * @return double  {arbitrary, 0.0}, {meter, 1.0}, {foot, 0.3048}, {us_foot, 0.3048006096012192}
         */
        TJH_SRS_EXPORT double LinearUnitInMeters(LinearUnits unit);

#pragma endregion LINEARUNITS_REGION

        /**
         * @brief 坐标系定义类型
         */
        enum SrsDefType
        {
            SrsDefType_LocalCartesian = 0, ///< 本地笛卡尔坐标系
            SrsDefType_ENU = 1,            ///< ENU坐标系
            SrsDefType_EPSG = 2,           ///< EPSG定义坐标系
            SrsDefType_WKT = 3,            ///< WKT定义坐标系
            SrsDefType_Proj4 = 4,          ///< Proj4定义坐标系
        };

        /**
         * @brief 坐标系定义基类
         * 
         */
        class TJH_SRS_EXPORT SrsDef
        {
        public:
            /**
             * @brief 获取坐标系定义类型
             */
            virtual SrsDefType type() const = 0;

            /**
             * @brief 坐标系定义转换为字符串
             * @return std::string 
             */
            virtual std::string toString() const = 0;

        protected:
            SrsDef();
        };

        /**
         * @brief 坐标系定义：本地笛卡尔坐标系
         */
        class TJH_SRS_EXPORT SrsDefLocalCartesian : public SrsDef
        {
        public:
            /**
             * @brief 线性单位
             */
            LinearUnits Units;

            /**
             * @brief 构造 Srs Def Local Cartesian 对象
             * @param u  线性单位，默认线性单位：任意
             */
            SrsDefLocalCartesian(LinearUnits u = arbitrary);

            /**
             * @brief 获取坐标系定义类型
             */
            SrsDefType type() const override { return SrsDefType::SrsDefType_LocalCartesian; }

            /**
             * @brief 坐标系定义转换为字符串
             * @details 不同线性单位的坐标定义字符串格式如下：
             *          "Local:unit=0";
             *          "Local:unit=Meter";
             *          "Local:unit=Foot";
             *          "Local:unit=Foot_US";
             */
            std::string toString() const override;
        };

        /**
         * @brief 坐标系定义：ENU(lat,lon)笛卡尔坐标系
         */
        class TJH_SRS_EXPORT SrsDefENU : public SrsDef
        {
        public:
            /**
             * @brief 构造 SrsDefENU 对象
             * @param lat 坐标原点：纬度
             * @param lon 坐标原点：经度
             */
            SrsDefENU(double lat, double lon);

            /**
             * @brief 构造 SrsDefENU 对象
             * @param _origin 坐标原点：经纬度坐标
             */
            SrsDefENU(const LonLat &_origin);

            /**
             * @brief 获取坐标系定义类型
             */
            SrsDefType type() const override { return SrsDefType::SrsDefType_ENU; }

            /**
             * @brief 坐标原点
             * @return const LonLat& 
             */
            const LonLat &origin() const { return m_origin; }

            /**
             * @brief 坐标系定义字符串
             * @return 坐标系定义字符串，格式：ENU(lat,lon), 精度：小数点后保留16位
             */
            std::string toString() const override;

        private:
            LonLat m_origin;
        };

        /**
         * @brief 坐标系定义：EPSG
         */
        class TJH_SRS_EXPORT SrsDefEPSG : public SrsDef
        {
        public:
            /**
             * @brief 构造 SrsDefEPSG 对象
             * @param epsg 
             */
            SrsDefEPSG(int epsg);

            /**
             * @brief 构造 SrsDefEPSG 对象
             * @param epsg 平面坐标系EPSG
             * @param vertical_epsg 覆盖垂直坐标系EPSG
             */
            SrsDefEPSG(int epsg, int vertical_epsg);

            /**
             * @brief 获取坐标系定义类型
             */
            SrsDefType type() const override { return SrsDefType::SrsDefType_EPSG; }

            /**
             * @brief 获取坐标系EPSG
             */
            int epsg() const { return m_epsg; }

            /**
             * @brief 获取覆盖垂直坐标系EPSG
             */
            int vertical_epsg() const { return m_vertical_epsg; }

            /**
             * @brief 坐标系定义字符串，格式类似：EPSG:4326
             */
            std::string toString() const override;

        private:
            int m_epsg;
            int m_vertical_epsg;
        };

        /**
         * @brief 坐标系定义：WKT
         */
        class TJH_SRS_EXPORT SrsDefWKT : public SrsDef
        {
        public:
            /**
             * @brief 构造 SrsDefWKT 对象
             * @param wkt 
             */
            SrsDefWKT(const std::string &wkt);

            /**
             * @brief 获取坐标系定义类型
             */
            SrsDefType type() const override { return SrsDefType::SrsDefType_WKT; }

            /**
             * @brief 获取WKT字符串
             */
            const std::string &wkt() const { return m_wkt; }

            /**
             * @brief 获取字符串定义，同wkt()
             */
            std::string toString() const override;

        private:
            std::string m_wkt;
        };

        /**
         * @brief 坐标系定义：Proj4
         */
        class TJH_SRS_EXPORT SrsDefProj4 : public SrsDef
        {
        public:
            /**
             * @brief 构造 SrsDefProj4 对象
             * @param proj4  proj4坐标系定义字符串
             */
            SrsDefProj4(const std::string &proj4);

            /**
             * @brief 获取坐标系定义类型
             */
            SrsDefType type() const override { return SrsDefType::SrsDefType_Proj4; }

            /**
             * @brief 获取proj4字符串
             */
            const std::string &proj4() const { return m_proj4; }

            /**
             * @brief 获取坐标系字符串定义，同proj4()
             */
            std::string toString() const override;

        private:
            std::string m_proj4;
        };

        /**
         * @brief WGS84（EPSG:4326）坐标系定义
         */
        const SrsDefEPSG SrsDef_WGS84 = SrsDefEPSG(WGS_84_EPSG);

        /**
         * @brief ECEF（EPSG:4978）坐标系定义
         */
        const SrsDefEPSG SrsDef_ECEF = SrsDefEPSG(ECEF_WGS84_EPSG);

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_SRSDEF_H_