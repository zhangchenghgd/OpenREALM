/**
 * @file TransformBase.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 坐标转换基类
 * @version 0.1
 * @date 2020-12-03
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_TRANSFORMBASE_H_
#define TJH_SRS_TRANSFORMBASE_H_

#include "Coord.h"
#include "Pose.h"

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 坐标转换基类
         */
        class TJH_SRS_EXPORT TransformBase
        {
        public:
            /**
             * @brief 点坐标转换
             * @param[in] src 待转换的点坐标
             * @return Coord3 返回转换后的点坐标，可通过Coord3.empty()判断转换成功
             */
            virtual Coord3 operator()(const Coord3 &src) const = 0;

            /**
             * @brief 相机Pose转换
             * @param[in] src 待转换的相机Pose
             * @return Pose 返回转换后的相机Pose，可通过Pose.empty()判断转换成功
             */
            virtual Pose operator()(const Pose &src) const = 0;

            /**
             * @brief 点坐标批量转换
             * @param[in] num_coord 点坐标数量
             * @param[in] src 原始坐标点数组
             * @param[out] dst 目标坐标点数组
             */
            virtual void transformCoord(size_t num_coord, const Coord3 *src, Coord3 *dst) const = 0;

            /**
             * @brief 相机的Pose坐标系批量转换
             * @param[in] num_pose 数量
             * @param[in] src 原始Pose数组
             * @param[out] dst 目标Pose数组
             */
            virtual void transformPose(size_t num_pose, const Pose *src, Pose *dst) const = 0;

            /**
             * @brief 打印转换信息
             */
            virtual void printTransform() const = 0;

        protected:
            TransformBase() {}
        };
    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_TRANSFORMBASE_H_
