/**
 * @file UTM.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef TJH_SRS_UTM_H_
#define TJH_SRS_UTM_H_

#include "SrsExport.h"
#include "Coord.h"
#include <string>

namespace TJH
{
    namespace SRS
    {
        class UTMPrivate;
        /**
         * @brief UTM 坐标系
         * 
         */
        class TJH_SRS_EXPORT UTM
        {
        public:
            /**
             * @brief 构造 UTM 对象
             * @param p_zone 分带号
             * @param p_north 是否北半球
             */
            UTM(int p_zone, bool p_north);

            /**
             * @brief 析构 UTM 对象
             */
            ~UTM();

            /**
             * @brief 分带
             */
            int zone() const;

            /**
             * @brief 是否北半球
             */
            bool north() const;

            /**
             * @brief 获取EPSG
             */
            int epsg() const;

            /**
             * @brief 获取UTM投影坐标系名称
             */
            std::string name() const;

            /**
             * @brief 坐标转换：经纬度 => UTM
             * @param[in] lla_coord 经纬度坐标，采用WGS84椭球
             * @return UTM坐标 
             */
            Coord3 fromLonLatAlt(const LonLatAlt &lla_coord) const;

            /**
             * @brief 坐标转换：UTM => 经纬度
             * @param[in] utm_coord UTM坐标
             * @return 经纬度坐标，采用UTM椭球
             */
            LonLatAlt toLonLatAlt(const Coord3 &utm_coord) const;

            /**
             * @brief 获取经纬度所在的UTM分度带
             * @param lon_lat 经纬度
             * @return UTM坐标系
             */
            static UTM locateUTM(const LonLat &lon_lat);

        private:
            int m_zone;
            bool m_north;
            UTMPrivate *m_UTMPrivate;
        };
    }
}

#endif // !TJH_SRS_UTM_H_