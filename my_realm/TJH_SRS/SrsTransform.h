/**
 * @file SrsTransform.h
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 不同坐标系的转换
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020 武汉天际航信息科技股份有限公司
 * 
 */

#ifndef TJH_SRS_SRSTRANSFORM_H_
#define TJH_SRS_SRSTRANSFORM_H_

#include "SrsDef.h"
#include "TransformBase.h"
#include <string>
#include <memory>

namespace TJH
{
    namespace SRS
    {
        class SrsTransform_PRIVATE;

        /**
         * @brief 坐标系转换
         */
        class TJH_SRS_EXPORT SrsTransform : public TransformBase
        {
        public:
            /**
             * @brief 构造 SrsTransform 对象
             */
            SrsTransform();

            /**
             * @brief  SrsTransform 坐标系转换构造函数
             * @param[in] src_srs 源坐标系定义
             * @param[in] dst_srs 目标坐标系定义
             */
            SrsTransform(const std::shared_ptr<SrsDef> &src_srs,
                        const std::shared_ptr<SrsDef> &dst_srs);

            /**
             * @brief 析构 SrsTransform 
             */
            ~SrsTransform();

            /**
             * @brief  Transform SRS 对象
             * @param[in] src_srs 源坐标系定义
             * @param[in] dst_srs 目标坐标系定义
             */
            bool setTransformSRS(const std::shared_ptr<SrsDef> &src_srs,
                                 const std::shared_ptr<SrsDef> &dst_srs);

            /**
             * @brief 获取源坐标系定义
             * @return std::string 源坐标系定义
             */
            const std::shared_ptr<SrsDef> srcSRS() const;

            /**
             * @brief 获取目标坐标系定义
             * @return std::string 目标坐标系定义
             */
            const std::shared_ptr<SrsDef> dstSRS() const;

            /**
             * @brief 坐标系转换逆转换
             * @return SrsTransform 
             */
            SrsTransform inverse() const;

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
             * @brief 点坐标批量转换
             * @param[in] num_coord 点坐标数量
             * @param[in] src 原始坐标点数组
             * @param[out] dst 目标坐标点数组
             */
            void transformCoord(size_t num_coord, const Coord3 *src,
                                Coord3 *dst) const override;

            /**
             * @brief 相机的Pose坐标系批量转换
             * @param[in] num_pose 数量
             * @param[in] src 原始Pose数组
             * @param[out] dst 目标Pose数组
             */
            void transformPose(size_t num_pose, const Pose *src,
                               Pose *dst) const override;

            /**
             * @brief 打印转换信息
             */
            void printTransform() const override;

        private:
            std::shared_ptr<SrsDef> src_srs;
            std::shared_ptr<SrsDef> dst_srs;
            SrsTransform_PRIVATE *m_SrsTransform_PRIVATE;
        };
    } // namespace SRS
} // namespace TJH

#endif // !TJH_SRS_SRSTRANSFORM_H_