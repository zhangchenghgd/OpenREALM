/**
 * @file RTSTransform.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief RTS坐标系转换
 * @version 0.1
 * @date 2020-12-02
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_RTSTRANSFORM_H_
#define TJH_SRS_RTSTRANSFORM_H_

#include "TransformBase.h"

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief  RTS转换
         * @details 转换公式： Dst = S * R * Src + T; \n
         *          该变换空间直角坐标系下的刚体变换；
         */
        class TJH_SRS_EXPORT RTSTransform : public TransformBase
        {
        public:
            double R[9];  ///< R [3x3] 旋转矩阵
            double T[3];  ///< T [3x1] T矩阵
            double S;     ///< S 缩放

        public:
            /**
             * @brief 构造 RTSTransform 对象
             */
            RTSTransform();

            /**
             * @brief 运算符重载
             * @return RTSTransform& 
             */
            RTSTransform& operator=(const RTSTransform& other);

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
             * @brief 坐标值RTS转换，坐标系为空间直角坐标系
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
            * @brief RTS参数解算
            * @param[in] num_coord 坐标数量
            * @param[in] src 原始坐标数组，坐标系为空间直角坐标系 
            * @param[in] dst 目标坐标数组，坐标系为空间直角坐标系
            * @param[out] residuals 残差数组
            * @return true 计算成功
            * @return false 计算失败
            */
            bool calcRTS(int num_coord, const Coord3 *src, const Coord3 *dst, double *residuals = 0);

            /**
             * @brief 逆向转换
             * @return RTSTransform 
             */
            RTSTransform inverse() const;

            /**
             * @brief 打印转换信息
             */
            void printTransform() const override;
        };

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_RTSTRANSFORM_H_
