/**
 * @file Pose.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 相机的Pose定义
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */
#ifndef TJH_SRS_POSE_H_
#define TJH_SRS_POSE_H_

//#include "SrsExport.h"
#include "Coord.h"

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 相机位置和旋转
         * 
         */
        class TJH_SRS_EXPORT Pose
        {
        public:
            /**
             * @brief 构造 Pose 对象
             */
            Pose();

            /**
             * @brief 构造 Pose 对象
             * @param[in] _center 相机中心
             * @param[in] _rotation 旋转矩阵 3x3
             */
            Pose(const Coord3 &_center, const double *_rotation);

            /**
             * @brief 构造 Pose 对象
             * @param x 相机中心X
             * @param y 相机中心Y
             * @param z 相机中心Z
             * @param omega 旋转Omega, 单位：角度
             * @param phi 旋转Phi, 单位：角度
             * @param kappa 旋转Kappa, 单位：角度
             */
            Pose(double x, double y, double z,
                 double omega, double phi, double kappa);

            /**
             * @brief  相机中心点
             * @return const Coord3& 相机中心点
             */
            const Coord3 &center() const;

            /**
             * @brief  设置相机中心点
             * @param[in] _center 相机中心点坐标
             */
            void setCenter(const Coord3 &_center);

            /**
             * @brief 获取旋转矩阵3x3，返回double数组，size = 9
             * @return const double* 旋转矩阵3x3
             */
            const double *rotation() const;

            /**
             * @brief  设置旋转矩阵3x3
             * @param[in] _rot 旋转矩阵3x3，double数组，size = 9
             */
            void setRotation(const double *_rot);

            /**
             * @brief 使用 Omega-Phi-Kappa 设置旋转
             * @param[in] omega 旋转角 omega, 单位：角度
             * @param[in] phi  旋转角 phi, 单位：角度
             * @param[in] kappa  旋转角 kappa, 单位：角度
             */
            void setRotation(double omega, double phi, double kappa);

            /**
             * @brief 获取 Omega-Phi-Kappa 旋转角度
             * @param[out] omega  旋转角 omega, 单位：角度
             * @param[out] phi 旋转角 phi, 单位：角度
             * @param[out] kappa   旋转角 kappa, 单位：角度
             */
            void getRotationOPK(double *omega, double *phi, double *kappa) const;

            /**
             * @brief 运算符'='赋值
             * @param other 
             * @return Pose& 
             */
            Pose &operator=(const Pose &other);

            /**
             * @brief Pose 是否为空
             * @return true 
             * @return false 
             */
            bool empty() const;

        private:
            Coord3 m_center;
            double m_rotation[9];
        };

        /**
         * @brief 转换：旋转矩阵3x3 => Omega-Phi-Kappa角元素
         * @param[in] R         旋转矩阵3x3，同ContextCapture中的旋转矩阵表示一致
         * @param[out] omega     omega，单位：角度
         * @param[out] phi       phi，单位：角度
         * @param[out] kappa     kappa，单位：角度
         */
        TJH_SRS_EXPORT void Matrix_To_EulerAngleOPK(
            const double *R,
            double *omega,
            double *phi,
            double *kappa);

        /**
         * @brief 转换：Omega-Phi-Kappa角元素 => 旋转矩阵3x3
         * @param[in] _omega    omega，单位：角度
         * @param[in] _phi      phi，单位：角度
         * @param[in] _kappa    kappa，单位：角度
         * @param[out] R        旋转矩阵3x3，同ContextCapture中的旋转矩阵表示一致
         */
        TJH_SRS_EXPORT void EulerAngleOPK_To_Matrix(
            double _omega,
            double _phi,
            double _kappa,
            double *R);

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_POSE_H_
