/**
 * @file FourParaTransform.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 四参数转换
 * @version 0.1
 * @date 2021-09-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef TJH_SRS_FOUR_PARA_H_
#define TJH_SRS_FOUR_PARA_H_

#include "TransformBase.h"

namespace TJH
{
    namespace SRS
    {
        /**
         * @brief 四参数转换
         * @details 平面坐标系下的坐标转换；\n
         *          - 平移参数：DX，DY 单位：米； \n
         *          - 旋转参数：R 单位：弧度； \n
         *          - 尺度参数：S，单位：ppm；\n
         *            初值为：0，这样即使进行了四参数转换，依然不改变坐标值；\n
         *         \n
         * @par 转换公式：
         * @code
         *        | X_dst |    | DX |             |  cos(R)   -sin(R) |   | X_src |  
		 *        |       | =  |    | + (1 + S) * |                   | * |       |  
         *        | Y_dst |    | DY |             |  sin(R)    cos(R) |   | Y_src |  
         *  @endcode
         */
        class TJH_SRS_EXPORT FourParaTransform
        {
        public:
            double DX;    ///< 平移参数 DX, 单位: 米
            double DY;    ///< 平移参数 DY, 单位: 米
            double R;     ///< 旋转参数 R, 单位: 弧度， （单位转换：秒 = RZ * 3600 * 180.0 / PI）
            double S;     ///< 尺度参数 K,  单位: 1，  （单位：ppm = S * 1000000）

        public:
            /**
             * @brief 构造 Bursa Transform 对象
             */
			FourParaTransform();

             /**
             * @brief 点坐标转换
             * @param[in] src 待转换的点坐标
             * @return Coord2 返回转换后的点坐标，可通过Coord2.empty()判断转换成功
             */
            Coord2 operator()(const Coord2 &src) const ;

            /**
             * @brief 坐标值四参数转换，坐标系为平面坐标系
             * @param[in] num_coord 坐标数量
             * @param[in] src 转换前点坐标数组
             * @param[out] dst 转换后点坐标数组
             */
            void transformCoord(size_t num_coord, const Coord2 *src, Coord2 *dst) const;


            /**
            * @brief 四参数解算；
            *        运用最小二乘计算四参数，公式为Ax=B，无需迭代
            * @param[in] num_coord 坐标数量
            * @param[in] src 原始坐标数组，坐标系为平面坐标系
            * @param[in] dst 目标坐标数组，坐标系为平面坐标系
            * @param[out] residuals 残差数组
            * @return true 计算成功
            * @return false 计算失败
            */
            bool calcFourPara(int num_coord, const Coord2 *src, const Coord2 *dst, 
				double *residuals = 0);

            /**
             * @brief 打印四参数信息
             */
            void printTransform() const;
        };

		/**
		 * @brief 四参数+高程拟合坐标转换
		 */
		class TJH_SRS_EXPORT FourParaHeightFitTransform : public FourParaTransform
		{
		public:
			double DZ;    ///< 高程平移参数 DZ, 单位: 米
		public:
			FourParaHeightFitTransform();
			
			/**
			 * @brief 点坐标转换
			 * @param[in] src 待转换的点坐标
			 * @return Coord3 返回转换后的点坐标，可通过Coord3.empty()判断转换成功
			 */
			Coord3 operator()(const Coord3 &src) const;
			
			/**
			 * @brief 坐标值四参数+高程拟合转换，坐标系为平面坐标系+高程
			 * @param[in] num_coord 坐标数量
			 * @param[in] src 转换前点坐标数组
			 * @param[out] dst 转换后点坐标数组
			 */
			void transformCoord(size_t num_coord, const Coord3 *src, Coord3 *dst) const;
			
			/**
			 * @brief 四参数+高程拟合解算；
			 *        运用最小二乘计算四参数和高程拟合，公式为Ax=B，无需迭代
			 * @param[in] num_coord 坐标数量
			 * @param[in] src 原始坐标数组，坐标系为平面坐标系
			 * @param[in] dst 目标坐标数组，坐标系为平面坐标系
			 * @param[out] horizon_residuals 水平方向残差数组
			 * @param[out] vertical_residuals 垂直方向残差数组
			 * @return true 计算成功
			 * @return false 计算失败
			 */
			bool calcFourParaHeightFit(int num_coord, const Coord3 *src, const Coord3 *dst, 
				double *horizon_residuals = 0, double *vertical_residuals = 0);
			
			/**
			 * @brief 打印四参数+高程拟合信息
			 */
			void printTransform() const;
		};

    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_FOUR_PARA_H_
