/**
 * @file BursaTransform.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 布尔莎七参数转换
 * @version 0.1
 * @date 2020-12-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef TJH_SRS_BRUSA_H_
#define TJH_SRS_BRUSA_H_

#include "TransformBase.h"

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 布尔莎七参数转换
         * @details 空间直角坐标系下的坐标系转换；\n
         *          - 平移参数：DX，DY，DZ 单位：米； \n
         *          - 旋转参数：RX，RY，RZ 单位：弧度； \n
         *          - 尺度参数：S，单位：ppm；\n
         *          初值为：0，这样即使进行了七参数转换，依然不改变坐标值；\n
         *         \n
         * @par 转换公式：
         * @code
         *        | X_dst |    | DX |             |  1   RZ -RY |   | X_src |  
         *        | Y_dst | =  | DY | + (1 + S) * | -RZ  1   RX | * | Y_src |  
         *        | Z_dst |    | DZ |             |  RY -RX  1  |   | Z_src |  
         * @endcode
         */
        class TJH_SRS_EXPORT BursaTransform : public TransformBase
        {
        public:
            double DX;    ///< 平移参数 DX, 单位: 米
            double DY;    ///< 平移参数 DY, 单位: 米
            double DZ;    ///< 平移参数 DZ, 单位: 米
            double RX;    ///< 旋转参数 RX, 单位: 弧度， （单位转换：秒 = RX * 3600 * 180.0 / PI）
            double RY;    ///< 旋转参数 RY, 单位: 弧度， （单位转换：秒 = RY * 3600 * 180.0 / PI）
            double RZ;    ///< 旋转参数 RZ, 单位: 弧度， （单位转换：秒 = RZ * 3600 * 180.0 / PI）
            double S;     ///< 尺度参数 K,  单位: 1， （单位：ppm = S * 1000000）

        public:
            /**
             * @brief 构造 Bursa Transform 对象
             */
            BursaTransform();

             /**
             * @brief 点坐标转换
             * @param[in] src 待转换的点坐标
             * @return Coord3 返回转换后的点坐标，可通过Coord3.empty()判断转换成功
             */
            Coord3 operator()(const Coord3 &src) const override;

            /**
             * @brief 相机Pose转换
             * @param[in] src 待转换的相机Pose
             * @return Pose 返回转换后的相机Pose，可通过Pose.empty()判断转换成功
             */
            Pose operator()(const Pose &src) const override;

            /**
             * @brief 坐标值七参数转换，坐标系为空间直角坐标系
             * @param[in] num_coord 坐标数量
             * @param[in] src 转换前点坐标数组
             * @param[out] dst 转换后点坐标数组
             */
            void transformCoord(size_t num_coord, const Coord3 *src, Coord3 *dst) const override;

            /**
             * @brief 相机的Pose坐标系转换
             * @param[in] num_pose 数量
             * @param[in] src 原始Pose数组
             * @param[out] dst 目标Pose数组
             */
            void transformPose(size_t num_pose, const Pose *src, Pose *dst) const override;

            /**
            * @brief Bursa-Wolf七参数解算；
            *        运用最小二乘计算七参数，公式为Ax=B，无需迭代
            * @param[in] num_coord 坐标数量
            * @param[in] src 原始坐标数组，坐标系为空间直角坐标系
            * @param[in] dst 目标坐标数组，坐标系为空间直角坐标系
            * @param[out] residuals 残差数组
            * @return true 计算成功
            * @return false 计算失败
            */
            bool calcBursa(int num_coord, const Coord3 *src, const Coord3 *dst, double *residuals = 0);

            /**
             * @brief 打印七参数信息
             */
            void printTransform() const override;
        };

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_BRUSA_H_
