/**
 * @file BursaTransform.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 布尔莎七参数转换
 * @version 0.1
 * @date 2020-12-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "BursaTransform.h"
#include <iostream>
#include <iomanip>
#include <math.h>
#include <Eigen/Eigen>

namespace TJH
{
    namespace SRS
    {
        BursaTransform::BursaTransform()
        {
            DX = 0.0;
            DY = 0.0;
            DZ = 0.0;
            RX = 0.0;
            RY = 0.0;
            RZ = 0.0;
            S = 0.0;
        }

        Coord3 BursaTransform::operator()(const Coord3 &src) const
        {
            ///< 转换坐标
            Coord3 src_coords[1];
            Coord3 dst_coords[1];
            src_coords[0] = src;
            transformCoord(1, &src_coords[0], &dst_coords[0]);
            Coord3 dst = dst_coords[0];
            return dst;
        }

        Pose BursaTransform::operator()(const Pose &src) const
        {
            ///< 转换Pose
            Pose src_poses[1];
            Pose dst_poses[1];
            src_poses[0] = src;
            transformPose(1, &src_poses[0], &dst_poses[0]);
            Pose dst = dst_poses[0];
            return dst;
        }

        void BursaTransform::transformCoord(size_t num_coord, const Coord3 *src, Coord3 *dst) const
        {
            // 初始化eigen矩阵
            Eigen::Matrix3d r_mat;
            Eigen::Vector3d d_vec, src_vec, dst_vec;

            //*******************************************
            // 构建旋转矩阵
            // |  1   RZ -RY |
            // | -RZ  1   RX |
            // |  RY -RX  1  |
            //*******************************************
            r_mat(0, 0) = 1.0;
            r_mat(0, 1) = RZ;
            r_mat(0, 2) = -RY;
            r_mat(1, 0) = -RZ;
            r_mat(1, 1) = 1.0;
            r_mat(1, 2) = RX;
            r_mat(2, 0) = RY;
            r_mat(2, 1) = -RX;
            r_mat(2, 2) = 1.0;

            d_vec = Eigen::Vector3d(DX, DY, DZ);

            for (int i = 0; i < num_coord; ++i)
            {
                // 源坐标
                src_vec = Eigen::Vector3d(src[i].x(), src[i].y(), src[i].z());
                // 目标坐标
                dst_vec = (1.0 + S) * r_mat * src_vec + d_vec;
                dst[i].set(dst_vec.x(), dst_vec.y(), dst_vec.z());
            }
        }

        void BursaTransform::transformPose(size_t num_pose, const Pose *src, Pose *dst) const
        {
            // 初始化eigen矩阵
            Eigen::Matrix3d r_mat, src_pose_r_mat, dst_pose_r_mat;
            Eigen::Vector3d d_vec, src_vec, dst_vec;
            r_mat(0, 0) = 1.0;
            r_mat(0, 1) = RZ;
            r_mat(0, 2) = -RY;
            r_mat(1, 0) = -RZ;
            r_mat(1, 1) = 1.0;
            r_mat(1, 2) = RX;
            r_mat(2, 0) = RY;
            r_mat(2, 1) = -RX;
            r_mat(2, 2) = 1.0;
            d_vec = Eigen::Vector3d(DX, DY, DZ);

            double dst_rot_arr[9];

            for (int i = 0; i < num_pose; ++i)
            {
                const Coord3 &src_center = src[i].center();

                src_pose_r_mat =
                    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(src[i].rotation());

                // 源坐标
                src_vec = Eigen::Vector3d(src_center.x(), src_center.y(), src_center.z());
                dst_vec = (1.0 + S) * r_mat * src_vec + d_vec;
                dst_pose_r_mat = src_pose_r_mat * r_mat.inverse();

                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dst_rot_arr, 3, 3) =
                    dst_pose_r_mat;

                dst[i].setCenter(Coord3(dst_vec.x(), dst_vec.y(), dst_vec.z()));
                dst[i].setRotation(&dst_rot_arr[0]);
            }
        }

        bool BursaTransform::calcBursa(int num_coord, const Coord3 *src,
                                       const Coord3 *dst, double *residuals)
        {
            // 点集数量少于3组，无法计算
            if (num_coord < 3)
                return false;

            // 计算坐标平均值
            Coord3 src_sum(0, 0, 0), dst_sum(0, 0, 0), src_mean(0, 0, 0), dst_mean(0, 0, 0);
            for (int i = 0; i < num_coord; i++)
            {
                src_sum = src_sum + src[i];
                dst_sum = dst_sum + dst[i];
            }
            src_mean = src_sum / num_coord;
            dst_mean = dst_sum / num_coord;

            Eigen::MatrixXd L(num_coord * 3, 1);
            Eigen::MatrixXd B(num_coord * 3, 7);

            for (int i = 0; i < num_coord; i++)
            {
                // 坐标重心化，减去坐标平均值
                Coord3 src_pt = src[i] - src_mean;
                Coord3 dst_pt = dst[i] - dst_mean;

                L(i * 3 + 0, 0) = dst_pt.x();    // - src[i].x();
                L(i * 3 + 1, 0) = dst_pt.y();    // - src[i].y();
                L(i * 3 + 2, 0) = dst_pt.z();    // - src[i].z();

                B(i * 3 + 0, 0) = 1.0;
                B(i * 3 + 0, 1) = 0.0;
                B(i * 3 + 0, 2) = 0.0;
                B(i * 3 + 0, 3) = src_pt.x();
                B(i * 3 + 0, 4) = 0.0;
                B(i * 3 + 0, 5) = -src_pt.z();
                B(i * 3 + 0, 6) = src_pt.y();

                B(i * 3 + 1, 0) = 0.0;
                B(i * 3 + 1, 1) = 1.0;
                B(i * 3 + 1, 2) = 0.0;
                B(i * 3 + 1, 3) = src_pt.y();
                B(i * 3 + 1, 4) = src_pt.z();
                B(i * 3 + 1, 5) = 0.0;
                B(i * 3 + 1, 6) = -src_pt.x();

                B(i * 3 + 2, 0) = 0.0;
                B(i * 3 + 2, 1) = 0.0;
                B(i * 3 + 2, 2) = 1.0;
                B(i * 3 + 2, 3) = src_pt.z();
                B(i * 3 + 2, 4) = -src_pt.y();
                B(i * 3 + 2, 5) = src_pt.x();
                B(i * 3 + 2, 6) = 0.0;
            }

            // 计算七参数
            Eigen::MatrixXd Para(7, 1);
            Para = (B.transpose() * B).inverse() * B.transpose() * L;

            // 七参数变量赋值
            double mean_DX = Para(0, 0);
            double mean_DY = Para(1, 0);
            double mean_DZ = Para(2, 0);
            S = Para(3, 0) - 1.0;
            RX = Para(4, 0) / Para(3, 0);
            RY = Para(5, 0) / Para(3, 0);
            RZ = Para(6, 0) / Para(3, 0);

            // 初始化eigen矩阵
            Eigen::Matrix3d mean_r_mat;
            Eigen::Vector3d mean_d_vec;
            mean_r_mat(0, 0) = 1.0;
            mean_r_mat(0, 1) = RZ; 
            mean_r_mat(0, 2) = -RY;
            mean_r_mat(1, 0) = -RZ;
            mean_r_mat(1, 1) = 1.0;
            mean_r_mat(1, 2) = RX;
            mean_r_mat(2, 0) = RY;
            mean_r_mat(2, 1) = -RX;
            mean_r_mat(2, 2) = 1.0;
            mean_d_vec = Eigen::Vector3d(mean_DX, mean_DY, mean_DZ);

            // 恢复坐标偏移
            Eigen::Vector3d _T = mean_d_vec +
                                 Eigen::Vector3d(dst_mean.x(), dst_mean.y(), dst_mean.z()) -
                                 (1.0 + S) * mean_r_mat * Eigen::Vector3d(src_mean.x(), src_mean.y(), src_mean.z());

            DX = _T[0];
            DY = _T[1];
            DZ = _T[2];

            if (residuals)
            {
                // 计算残差
                Coord3 *dst_temp = new Coord3[num_coord];
                transformCoord(num_coord, src, dst_temp);
                for (int i = 0; i < num_coord; i++)
                {
                    residuals[i] = (dst[i] - dst_temp[i]).norm();
                }
                delete dst_temp;
            }

            return true;
        }

        void BursaTransform::printTransform() const
        {
            ///< 打印七参数信息
            std::cout << std::setprecision(16);
            std::cout << "Seven parameters of the Bursa-Wolf transformation:" << std::endl
                      << " -- DX: " << DX << " (m)" << std::endl
                      << " -- DY: " << DY << " (m)" << std::endl
                      << " -- DZ: " << DZ << " (m)" << std::endl
                      << " -- RX: " << RX * 3600 * 180.0 / M_PI << " (secs)" << std::endl
                      << " -- RY: " << RY * 3600 * 180.0 / M_PI << " (secs)" << std::endl
                      << " -- RZ: " << RZ * 3600 * 180.0 / M_PI << " (secs)" << std::endl
                      << " -- S:  " << S * 1000000 << " (ppm)" << std::endl;
        }

    } // namespace SRS
} // namespace TJH
