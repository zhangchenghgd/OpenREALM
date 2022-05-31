/**
 * @file FourParaTransform.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 四参数转换
 * @version 0.1
 * @date 2020-09-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "FourParaTransform.h"
#include <iostream>
#include <iomanip>
#include <math.h>
#include <Eigen/Eigen>

namespace TJH
{
	namespace SRS
	{
		FourParaTransform::FourParaTransform()
		{
			DX = 0.0;
			DY = 0.0;
			R = 0.0;
			S = 0.0;
		}

		Coord2 FourParaTransform::operator()(const Coord2 &src) const
		{
			///< 转换坐标
			Coord2 src_coords[1];
			Coord2 dst_coords[1];
			src_coords[0] = src;
			transformCoord(1, &src_coords[0], &dst_coords[0]);
			Coord2 dst = dst_coords[0];
			return dst;
		}


		void FourParaTransform::transformCoord(size_t num_coord, const Coord2 *src, Coord2 *dst) const
		{
			// 初始化eigen矩阵
			Eigen::Matrix2d r_mat;
			Eigen::Vector2d d_vec, src_vec, dst_vec;

			//*******************************************
			// 构建旋转矩阵
			// | cos_a -sin_a |
			// | sin_a  cos_a |
			//*******************************************
			r_mat(0, 0) = cos(R);
			r_mat(0, 1) = -sin(R);
			r_mat(1, 0) = sin(R);
			r_mat(1, 1) = cos(R);

			d_vec = Eigen::Vector2d(DX, DY);

			for (int i = 0; i < num_coord; ++i)
			{
				// 源坐标
				src_vec = Eigen::Vector2d(src[i].x(), src[i].y());
				// 目标坐标
				dst_vec = (1.0 + S) * r_mat * src_vec + d_vec;
				dst[i].set(dst_vec.x(), dst_vec.y());
			}
		}

		bool FourParaTransform::calcFourPara(int num_coord, const Coord2 *src,
			const Coord2 *dst, double *residuals)
		{
			// 点集数量少于3组，无法计算
			if (num_coord < 2)
				return false;

			Eigen::MatrixXd L(num_coord * 2, 1);
			Eigen::MatrixXd B(num_coord * 2, 4);

			for (int i = 0; i < num_coord; i++)
			{
				// 坐标重心化，减去坐标平均值
				Coord2 src_pt = src[i];
				Coord2 dst_pt = dst[i];

				L(i * 2 + 0, 0) = dst_pt.x() - src[i].x();
				L(i * 2 + 1, 0) = dst_pt.y() - src[i].y();

				B(i * 2 + 0, 0) = 1.0;
				B(i * 2 + 0, 1) = 0.0;
				B(i * 2 + 0, 2) = src_pt.x();
				B(i * 2 + 0, 3) = -src_pt.y();

				B(i * 2 + 1, 0) = 0.0;
				B(i * 2 + 1, 1) = 1.0;
				B(i * 2 + 1, 2) = src_pt.y();
				B(i * 2 + 1, 3) = src_pt.x();

			}

			// 计算四参数
			Eigen::MatrixXd Para(4, 1);
			Para = (B.transpose() * B).inverse() * B.transpose() * L;

			// 七参数变量赋值
			double mean_DX = Para(0, 0);
			double mean_DY = Para(1, 0);
			double a = Para(2, 0);
			double b = Para(3, 0);

			R = atan(b / (a + 1));
			S = b / sin(R) - 1;

			// 初始化eigen矩阵
			Eigen::Matrix2d mean_r_mat;
			Eigen::Vector2d mean_d_vec;
			mean_r_mat(0, 0) = cos(R);
			mean_r_mat(0, 1) = -sin(R);
			mean_r_mat(1, 0) = sin(R);
			mean_r_mat(1, 1) = cos(R);
			mean_d_vec = Eigen::Vector2d(mean_DX, mean_DY);

			Eigen::Vector2d _T = mean_d_vec;

			DX = _T[0];
			DY = _T[1];

			if (residuals)
			{
				// 计算残差
				Coord2 *dst_temp = new Coord2[num_coord];
				transformCoord(num_coord, src, dst_temp);
				for (int i = 0; i < num_coord; i++)
				{
					residuals[i] = (dst[i] - dst_temp[i]).norm();
				}
				delete dst_temp;
			}

			return true;
		}

		void FourParaTransform::printTransform() const
		{
			///< 打印七参数信息
			std::cout << std::setprecision(16);
			std::cout << "Four parameters of the four-para transformation:" << std::endl
				<< " -- DX: " << DX << " (m)" << std::endl
				<< " -- DY: " << DY << " (m)" << std::endl
				<< " -- R:  " << R * 3600 * 180.0 / M_PI << " (secs)" << std::endl
				<< " -- S:  " << S * 1000000 << " (ppm)" << std::endl;
		}

		FourParaHeightFitTransform::FourParaHeightFitTransform()
			:FourParaTransform(), DZ(0.0)
		{
		}

		Coord3 FourParaHeightFitTransform::operator()(const Coord3 & src) const
		{
			///< 转换坐标
			Coord3 src_coords[1];
			Coord3 dst_coords[1];
			src_coords[0] = src;
			this->transformCoord(1, &src_coords[0], &dst_coords[0]);
			Coord3 dst = dst_coords[0];
			return dst;
		}

		void FourParaHeightFitTransform::transformCoord(size_t num_coord, const Coord3 * src, Coord3 * dst) const
		{
			// 初始化eigen矩阵
			Eigen::Matrix2d r_mat;
			Eigen::Vector2d d_vec, src_vec, dst_vec;

			//*******************************************
			// 构建旋转矩阵
			// | cos_a -sin_a |
			// | sin_a  cos_a |
			//*******************************************
			r_mat(0, 0) = cos(R);
			r_mat(0, 1) = -sin(R);
			r_mat(1, 0) = sin(R);
			r_mat(1, 1) = cos(R);

			d_vec = Eigen::Vector2d(DX, DY);

			for (int i = 0; i < num_coord; ++i)
			{
				// 源坐标
				src_vec = Eigen::Vector2d(src[i].x(), src[i].y());
				// 目标坐标
				dst_vec = (1.0 + S) * r_mat * src_vec + d_vec;
				double dst_z = src[i].z() + DZ;
				dst[i].set(dst_vec.x(), dst_vec.y(), dst_z);
			}
		}

		bool FourParaHeightFitTransform::calcFourParaHeightFit(int num_coord, const Coord3 * src,
			const Coord3 * dst, double *horizon_residuals, double *vertical_residuals)
		{
			// 点集数量少于3组，无法计算
			if (num_coord < 2)
				return false;
			Coord2* src_crd2 = new Coord2[num_coord];
			Coord2* dst_crd2 = new Coord2[num_coord];
			double sum_dz = 0.0;
			for (int i = 0; i < num_coord; ++i)
			{
				src_crd2[i] = Coord2(src[i].x(), src[i].y());
				dst_crd2[i] = Coord2(dst[i].x(), dst[i].y());
				sum_dz += (dst[i].z() - src[i].z());
			}

			bool suc = false;

			// 计算四参数
			if (FourParaTransform::calcFourPara(num_coord, src_crd2, dst_crd2, horizon_residuals))
			{
				// 计算高程改正
				DZ = sum_dz / num_coord;

				if (vertical_residuals)
				{
					// 计算残差
					Coord3 *dst_temp = new Coord3[num_coord];
					transformCoord(num_coord, src, dst_temp);
					for (int i = 0; i < num_coord; i++)
					{
						vertical_residuals[i] = dst[i].z() - dst_temp[i].z();
					}
					delete dst_temp;
				}
				suc = true;
			}

			delete[] src_crd2;
			delete[] dst_crd2;

			return suc;
		}

		void FourParaHeightFitTransform::printTransform() const
		{
			///< 打印七参数信息
			std::cout << std::setprecision(16);
			std::cout << "Parameters of the four-para with height fit transformation:" << std::endl
				<< " -- DX: " << DX << " (m)" << std::endl
				<< " -- DY: " << DY << " (m)" << std::endl
				<< " -- R:  " << R * 3600 * 180.0 / M_PI << " (secs)" << std::endl
				<< " -- S:  " << S * 1000000 << " (ppm)" << std::endl
				<< " -- DZ: " << DZ << " (m)" << std::endl;
		}

	} // namespace SRS
} // namespace TJH
