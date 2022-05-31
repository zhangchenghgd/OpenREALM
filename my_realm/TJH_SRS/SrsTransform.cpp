/**
 * @file OGRTransform.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "SrsTransform.h"
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogr_srs_api.h>
#include <ogr_api.h>
#include <ogr_core.h>
#include <Eigen/Eigen>
#include <iostream>
#include "RTSTransform.h"
#include "ENU.h"

namespace TJH
{
    namespace SRS
    {
        class SrsTransform_PRIVATE
        {
            friend class SrsTransform;

        private:
            OGRSpatialReference *SRC_SR; ///< 源坐标系下的OGR空间参考，若源坐标系为ENU，此处为WGS84(EPSG:4326)
            OGRSpatialReference *DST_SR; ///< 目标坐标系下的OGR空间参考，若目标坐标系为ENU，此处为WGS84(EPSG:4326)

            OGRSpatialReference *SRC_Horiz_SR; ///< 源坐标系下的OGR空间参考：平面坐标系
            OGRSpatialReference *DST_Horiz_SR; ///< 目标坐标系下的OGR空间参考：平面坐标系
            OGRSpatialReference *SRC_Vert_SR;  ///< 源坐标系下的OGR空间参考：垂直坐标系
            OGRSpatialReference *DST_Vert_SR;  ///< 目标坐标系下的OGR空间参考：垂直坐标系

            OGRCoordinateTransformation *OGR_CT; ///< OGR坐标系转换
            ENU *SRC_ENU;                        ///< 若源坐标系为ENU
            ENU *DST_ENU;                        ///< 若目标坐标系为ENU

            SrsTransform_PRIVATE()
            {
                SRC_SR = new OGRSpatialReference;
                DST_SR = new OGRSpatialReference;

                SRC_Horiz_SR = new OGRSpatialReference;
                DST_Horiz_SR = new OGRSpatialReference;
                SRC_Vert_SR = new OGRSpatialReference;
                DST_Vert_SR = new OGRSpatialReference;

                OGR_CT = NULL;
                SRC_ENU = new ENU;
                DST_ENU = new ENU;
            }

            ~SrsTransform_PRIVATE()
            {
                if (OGR_CT)
                {
                    OGRCoordinateTransformation::DestroyCT(OGR_CT);
                }
                OGRSpatialReference::DestroySpatialReference(SRC_SR);
                OGRSpatialReference::DestroySpatialReference(DST_SR);
                OGRSpatialReference::DestroySpatialReference(SRC_Horiz_SR);
                OGRSpatialReference::DestroySpatialReference(DST_Horiz_SR);
                OGRSpatialReference::DestroySpatialReference(SRC_Vert_SR);
                OGRSpatialReference::DestroySpatialReference(DST_Vert_SR);

                delete SRC_ENU;
                delete DST_ENU;
            }

            void clear()
            {
                if (OGR_CT)
                {
                    OGRCoordinateTransformation::DestroyCT(OGR_CT);
                    OGR_CT = NULL;
                }

                if (SRC_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(SRC_SR);
                    SRC_SR = NULL;
                }
                if (DST_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(DST_SR);
                    SRC_SR = NULL;
                }

                if (SRC_Horiz_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(SRC_Horiz_SR);
                    SRC_Horiz_SR = NULL;
                }
                if (DST_Horiz_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(DST_Horiz_SR);
                    DST_Horiz_SR = NULL;
                }
                if (SRC_Vert_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(SRC_Vert_SR);
                    SRC_Vert_SR = NULL;
                }
                if (DST_Vert_SR)
                {
                    OGRSpatialReference::DestroySpatialReference(DST_Vert_SR);
                    DST_Vert_SR = NULL;
                }

                if (SRC_ENU)
                {
                    delete SRC_ENU;
                    SRC_ENU = NULL;
                }
                if (DST_ENU)
                {
                    delete DST_ENU;
                    DST_ENU = NULL;
                }

                SRC_SR = new OGRSpatialReference;
                DST_SR = new OGRSpatialReference;
                SRC_Horiz_SR = new OGRSpatialReference;
                DST_Horiz_SR = new OGRSpatialReference;
                SRC_Vert_SR = new OGRSpatialReference;
                DST_Vert_SR = new OGRSpatialReference;
                OGR_CT = NULL;
                SRC_ENU = new ENU;
                DST_ENU = new ENU;
            }

            bool createCT()
            {
                if (OGR_CT)
                {
                    OGRCoordinateTransformation::DestroyCT(OGR_CT);
                    OGR_CT = NULL;
                }

                if (SRC_SR && DST_SR)
                {
                    OGR_CT = OGRCreateCoordinateTransformation(SRC_SR, DST_SR);
                    if (OGR_CT)
                        return true;
                }

                return false;
            }

            bool transformCoord(size_t num_coord,
                                const Coord3 *src,
                                Coord3 *dst) const
            {
                if (!OGR_CT || 0 == num_coord)
                    return false;

                double *x_arry = new double[num_coord];
                double *y_arry = new double[num_coord];
                double *z_arry = new double[num_coord];
                int *pabSuccess = new int[num_coord];
                for (int i = 0; i < num_coord; ++i)
                {
                    Coord3 src_coord = src[i];

                    if (SRC_ENU->valid())
                    {
                        src_coord = SRC_ENU->coordToLonLatAlt(src_coord);
                    }

                    x_arry[i] = src_coord.x();
                    y_arry[i] = src_coord.y();
                    z_arry[i] = src_coord.z();
                }

#if (GDAL_VERSION_MAJOR == 3)
                int suc = OGR_CT->Transform((int)num_coord,
                                            x_arry, y_arry,
                                            z_arry, pabSuccess);
#elif (GDAL_VERSION_MAJOR == 2)
                int suc = OGR_CT->TransformEx((int)num_coord,
                                              x_arry, y_arry,
                                              z_arry, pabSuccess);
#endif // !(GDAL_VERSION_MAJOR == 3)

                int suc_num = 0;
                for (size_t i = 0; i < num_coord; ++i)
                {
                    if (pabSuccess[i] == FALSE)
                    {
                        continue;
                    }
                    Coord3 dst_coord = Coord3(x_arry[i], y_arry[i], z_arry[i]);
                    if (DST_ENU->valid())
                    {
                        dst_coord = DST_ENU->coordFromLonLatAlt(dst_coord);
                    }
                    dst[i] = dst_coord;
                    ++suc_num;
                }

                delete[] x_arry;
                delete[] y_arry;
                delete[] z_arry;
                delete[] pabSuccess;

                return suc_num > 0;
            }

            bool transformPose(size_t num_pose,
                               const Pose *src,
                               Pose *dst) const
            {
                if (!OGR_CT || 0 == num_pose)
                    return false;

                int suc_num = 0;

                /// 开始转换所有相机的Rotation
                /// 第1步：判断原始和目标坐标系是否为地理坐标系（经纬度坐标）
                ENU *enu_src_i = NULL;
                ENU *enu_dst_i = NULL;
                if (!SRC_ENU->valid() && SRC_SR->IsGeographic())
                {
                    double a1 = SRC_SR->GetSemiMajor();
                    double b1 = SRC_SR->GetSemiMinor();
                    enu_src_i = new ENU(GeoEllipsoid(a1, b1));
                }
                if (!DST_ENU->valid() && DST_SR->IsGeographic())
                {
                    double a2 = DST_SR->GetSemiMajor();
                    double b2 = DST_SR->GetSemiMinor();
                    enu_dst_i = new ENU(GeoEllipsoid(a2, b2));
                }

                const Coord3 Oc(0.0, 0.0, 0.0);
                const Coord3 Ac(0.1, 0.0, 0.0);
                const Coord3 Bc(0.0, 0.1, 0.0);
                const Coord3 Cc(0.0, 0.0, 0.1);

                for (size_t i = 0; i < num_pose; ++i)
                {
                    Coord3 rts_src[4];
                    Coord3 rts_dst[4];

                    Coord3 coord_src_axis[4];
                    Coord3 coord_dst_axis[4];
                    if (enu_src_i)
                    {
                        const Coord3 &src_lla = src[i].center();
                        enu_src_i->setOrigin(src_lla);

                        coord_src_axis[0] = enu_src_i->coordToLonLatAlt(Oc); // = src_lla
                        coord_src_axis[1] = enu_src_i->coordToLonLatAlt(Ac);
                        coord_src_axis[2] = enu_src_i->coordToLonLatAlt(Bc);
                        coord_src_axis[3] = enu_src_i->coordToLonLatAlt(Cc);

                        rts_src[0] = Oc;
                        rts_src[1] = Ac;
                        rts_src[2] = Bc;
                        rts_src[3] = Cc;
                    }
                    else
                    {
                        coord_src_axis[0] = Oc + src[i].center();
                        coord_src_axis[1] = Ac + src[i].center();
                        coord_src_axis[2] = Bc + src[i].center();
                        coord_src_axis[3] = Cc + src[i].center();

                        rts_src[0] = coord_src_axis[0];
                        rts_src[1] = coord_src_axis[1];
                        rts_src[2] = coord_src_axis[2];
                        rts_src[3] = coord_src_axis[3];
                    }

                    bool suc = this->transformCoord(4, coord_src_axis, coord_dst_axis);
                    if (!suc)
                    {
                        continue;
                    }

                    if (enu_dst_i)
                    {
                        const Coord3 &dst_lla = coord_dst_axis[0];
                        enu_dst_i->setOrigin(dst_lla);
                        rts_dst[0] = enu_dst_i->coordFromLonLatAlt(coord_dst_axis[0]); // = (0.0, 0.0, 0.0)
                        rts_dst[1] = enu_dst_i->coordFromLonLatAlt(coord_dst_axis[1]);
                        rts_dst[2] = enu_dst_i->coordFromLonLatAlt(coord_dst_axis[2]);
                        rts_dst[3] = enu_dst_i->coordFromLonLatAlt(coord_dst_axis[3]);
                    }
                    else
                    {
                        rts_dst[0] = coord_dst_axis[0];
                        rts_dst[1] = coord_dst_axis[1];
                        rts_dst[2] = coord_dst_axis[2];
                        rts_dst[3] = coord_dst_axis[3];
                    }

                    double resdual[4];
                    RTSTransform rts;
                    rts.calcRTS(4, rts_src, rts_dst, &resdual[0]);

                    Pose rts_src_pose;
                    rts_src_pose.setCenter(rts_src[0]);
                    rts_src_pose.setRotation(src[i].rotation());

                    Pose dst_pose = rts(rts_src_pose);
                    dst_pose.setCenter(coord_dst_axis[0]);

                    dst[i] = dst_pose;

                    ++suc_num;
                }

                if (enu_src_i)
                {
                    delete enu_src_i;
                }
                if (enu_dst_i)
                {
                    delete enu_dst_i;
                }

                return suc_num > 0;
            }
        };

        SrsTransform::SrsTransform()
        {
            m_SrsTransform_PRIVATE = new SrsTransform_PRIVATE;
        }

        SrsTransform::SrsTransform(const std::shared_ptr<SrsDef> &src_srs,
                                   const std::shared_ptr<SrsDef> &dst_srs)
            : SrsTransform()
        {
            setTransformSRS(src_srs, dst_srs);
        }

        SrsTransform::~SrsTransform()
        {
            delete m_SrsTransform_PRIVATE;
			src_srs = NULL;
			dst_srs = NULL;
        }

        bool SrsTransform::setTransformSRS(const std::shared_ptr<SrsDef> &p_src_srs,
                                           const std::shared_ptr<SrsDef> &p_dst_srs)
        {
            m_SrsTransform_PRIVATE->clear();

            std::shared_ptr<SrsDef> t_src_srs;
            std::shared_ptr<SrsDef> t_dst_srs;

            if (p_src_srs->type() == SrsDefType_EPSG)
            {
                const SrsDefEPSG &src_def_epsg = dynamic_cast<const SrsDefEPSG &>(*p_src_srs);
                int horiz_epsg = src_def_epsg.epsg();
                int vert_epsg = src_def_epsg.vertical_epsg();
                if (vert_epsg > 0)
                {
                    if (OGRERR_NONE == m_SrsTransform_PRIVATE->SRC_Horiz_SR->importFromEPSG(horiz_epsg) &&
                        OGRERR_NONE == m_SrsTransform_PRIVATE->SRC_Vert_SR->importFromEPSG(vert_epsg))
                    {
                        if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->SetCompoundCS(
                                               src_def_epsg.toString().c_str(),
                                               m_SrsTransform_PRIVATE->SRC_Horiz_SR,
                                               m_SrsTransform_PRIVATE->SRC_Vert_SR))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->importFromEPSG(horiz_epsg))
                    {
                        return false;
                    }
                }
                src_srs = std::make_shared<SrsDefEPSG>(src_def_epsg);
            }
            else if (p_src_srs->type() == SrsDefType_WKT)
            {
                const SrsDefWKT &src_def_wkt = dynamic_cast<const SrsDefWKT &>(*p_src_srs);
#if (GDAL_VERSION_MAJOR == 3)
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->importFromWkt(
                                       src_def_wkt.toString().c_str()))
#elif (GDAL_VERSION_MAJOR == 2)
                char wkt_buf[2048];
                strcpy(wkt_buf, src_def_wkt.toString().c_str());
                char *str_wkt_ptr = (char *)wkt_buf;
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->importFromWkt(&str_wkt_ptr))
#endif // !(GDAL_VERSION_MAJOR == 2)
                {
                    return false;
                }
                src_srs = std::make_shared<SrsDefWKT>(src_def_wkt);
            }
            else if (p_src_srs->type() == SrsDefType_Proj4)
            {
                const SrsDefProj4 &src_def_prj = dynamic_cast<const SrsDefProj4 &>(*p_src_srs);
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->importFromProj4(src_def_prj.toString().c_str()))
                {
                    return false;
                }
                src_srs = std::make_shared<SrsDefProj4>(src_def_prj);
            }
            else if (p_src_srs->type() == SrsDefType_ENU)
            {
                const SrsDefENU &srs_def_enu =
                    dynamic_cast<const SrsDefENU &>(*p_src_srs);
                m_SrsTransform_PRIVATE->SRC_ENU->setOrigin(Coord3(srs_def_enu.origin()));
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->importFromEPSG(WGS_84_EPSG))
                {
                    return false;
                }
                src_srs = std::make_shared<SrsDefENU>(srs_def_enu);
            }
            else if (p_src_srs->type() == SrsDefType_LocalCartesian)
            {
                const SrsDefLocalCartesian &srs_def_local =
                    dynamic_cast<const SrsDefLocalCartesian &>(*p_src_srs);
                std::string unit_str = LinearUnitString(srs_def_local.Units);
                double unit_in_meters = LinearUnitInMeters(srs_def_local.Units);
                if (unit_str.length() > 0 && unit_in_meters > 0)
                {
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->SetLocalCS(
                                           srs_def_local.toString().c_str()))
                    {
                        return false;
                    }
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->SRC_SR->SetLinearUnits(
                                           unit_str.c_str(), unit_in_meters))
                    {
                        return false;
                    }
                }
                src_srs = std::make_shared<SrsDefLocalCartesian>(srs_def_local);
            }
            else
            {
                return false;
            }

            if (p_dst_srs->type() == SrsDefType_EPSG)
            {
                const SrsDefEPSG &dst_def_epsg = dynamic_cast<const SrsDefEPSG &>(*p_dst_srs);

                int horiz_epsg = dst_def_epsg.epsg();
                int vert_epsg = dst_def_epsg.vertical_epsg();
                if (vert_epsg > 0)
                {
                    if (OGRERR_NONE == m_SrsTransform_PRIVATE->DST_Horiz_SR->importFromEPSG(horiz_epsg) &&
                        OGRERR_NONE == m_SrsTransform_PRIVATE->DST_Vert_SR->importFromEPSG(vert_epsg))
                    {
                        if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->SetCompoundCS(
                                               dst_def_epsg.toString().c_str(),
                                               m_SrsTransform_PRIVATE->DST_Horiz_SR,
                                               m_SrsTransform_PRIVATE->DST_Vert_SR))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->importFromEPSG(horiz_epsg))
                    {
                        return false;
                    }
                }
                dst_srs = std::make_shared<SrsDefEPSG>(dst_def_epsg);
            }
            else if (p_dst_srs->type() == SrsDefType_WKT)
            {
                const SrsDefWKT &dst_srs_wkt = dynamic_cast<const SrsDefWKT &>(*p_dst_srs);
#if (GDAL_VERSION_MAJOR == 3)
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->importFromWkt(dst_srs_wkt.toString().c_str()))
#elif (GDAL_VERSION_MAJOR == 2)
                char wkt_buf[2048];
                strcpy(wkt_buf, dst_srs_wkt.toString().c_str());
                char *str_wkt_ptr = (char *)wkt_buf;
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->importFromWkt(&str_wkt_ptr))
#endif // !(GDAL_VERSION_MAJOR == 3)
                {
                    return false;
                }
                dst_srs = std::make_shared<SrsDefWKT>(dst_srs_wkt);
            }
            else if (p_dst_srs->type() == SrsDefType_Proj4)
            {
                const SrsDefProj4 &dst_srs_prj = dynamic_cast<const SrsDefProj4 &>(*p_dst_srs);
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->importFromProj4(dst_srs_prj.toString().c_str()))
                {
                    return false;
                }
                dst_srs = std::make_shared<SrsDefProj4>(dst_srs_prj);
            }
            else if (p_dst_srs->type() == SrsDefType_ENU)
            {
                const SrsDefENU &srs_def_enu =
                    dynamic_cast<const SrsDefENU &>(*p_dst_srs);
                m_SrsTransform_PRIVATE->DST_ENU->setOrigin(Coord3(srs_def_enu.origin()));
                if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->importFromEPSG(WGS_84_EPSG))
                {
                    return false;
                }
                dst_srs = std::make_shared<SrsDefENU>(srs_def_enu);
            }
            else if (p_dst_srs->type() == SrsDefType_LocalCartesian)
            {
                const SrsDefLocalCartesian &dst_def_local =
                    dynamic_cast<const SrsDefLocalCartesian &>(*p_dst_srs);
                std::string unit_str = LinearUnitString(dst_def_local.Units);
                double unit_in_meters = LinearUnitInMeters(dst_def_local.Units);
                if (unit_str.length() > 0 && unit_in_meters > 0)
                {
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->SetLocalCS(
                                           dst_def_local.toString().c_str()))
                    {
                        return false;
                    }
                    if (OGRERR_NONE != m_SrsTransform_PRIVATE->DST_SR->SetLinearUnits(
                                           unit_str.c_str(), unit_in_meters))
                    {
                        return false;
                    }
                }
                dst_srs = std::make_shared<SrsDefLocalCartesian>(dst_def_local);
            }
            else
            {
                return false;
            }
#if (GDAL_VERSION_MAJOR == 3)
            m_SrsTransform_PRIVATE->SRC_SR->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            m_SrsTransform_PRIVATE->DST_SR->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
#endif // !(GDAL_VERSION_MAJOR == 3)

            bool suc = m_SrsTransform_PRIVATE->createCT();

            return suc;
        }

        const std::shared_ptr<SrsDef> SrsTransform::srcSRS() const
        {
            return src_srs;
        }

        const std::shared_ptr<SrsDef> SrsTransform::dstSRS() const
        {
            return dst_srs;
        }

        SrsTransform SrsTransform::inverse() const
        {
            SrsTransform transform;
            transform.setTransformSRS(dst_srs, src_srs);
            return transform;
        }

        Coord3 SrsTransform::operator()(const Coord3 &src) const
        {
            if (src_srs == NULL || dst_srs == NULL)
            {
                return Coord3();
            }
            Coord3 src_coords[1];
            Coord3 dst_coords[1];
            src_coords[0] = src;
            transformCoord(1, &src_coords[0], &dst_coords[0]);
            Coord3 dst = dst_coords[0];
            return dst;
        }

        Pose SrsTransform::operator()(const Pose &src) const
        {
            if (src_srs == NULL || dst_srs == NULL)
            {
                return Pose();
            }
            Pose src_poses[1];
            Pose dst_poses[1];
            src_poses[0] = src;
            transformPose(1, &src_poses[0], &dst_poses[0]);
            Pose dst = dst_poses[0];
            return dst;
        }

        void SrsTransform::transformCoord(size_t num_coord,
                                          const Coord3 *src,
                                          Coord3 *dst) const
        {
            if (src_srs == NULL || dst_srs == NULL)
                return;
            if (src_srs->toString().compare(dst_srs->toString()) == 0)
            {
                for (size_t i = 0; i < num_coord; ++i)
                {
                    dst[i] = src[i];
                }
            }
            m_SrsTransform_PRIVATE->transformCoord(num_coord, src, dst);
        }

        void SrsTransform::transformPose(size_t num_pose,
                                         const Pose *src,
                                         Pose *dst) const
        {
            if (src_srs == NULL || dst_srs == NULL)
                return;
            if (src_srs->toString().compare(dst_srs->toString()) == 0)
            {
                for (size_t i = 0; i < num_pose; ++i)
                {
                    dst[i] = src[i];
                }
            }
            m_SrsTransform_PRIVATE->transformPose(num_pose, src, dst);
        }

        void SrsTransform::printTransform() const
        {
            std::cout
                << "SRS transformation:\n"
                << " -- src srs: " << src_srs->toString() << "\n"
                << " -- dst srs: " << dst_srs->toString() << "\n"
                << std::endl;
        }
    } // namespace SRS
} // namespace TJH