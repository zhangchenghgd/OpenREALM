#include "RTSTransform.h"
#include <iostream>
#include <iomanip>
#include <Eigen/Eigen>

namespace TJH
{
    namespace SRS
    {
        RTSTransform::RTSTransform()
        {
            // R 设为单位矩阵
            memset(R, 0, 9 * sizeof(double));
            R[0] = R[4] = R[8] = 1.0;

            // T 设为 [0, 0, 0]
            memset(T, 0, 3 * sizeof(double));

            // S 置为 1
            S = 1.0;
        }

        RTSTransform& RTSTransform::operator=(const RTSTransform& other)
        {
            this->S = other.S;
            memcpy(this->R, other.R, 9 * sizeof(double));
            memcpy(this->T, other.T, 3 * sizeof(double));
            return *this;
        }

        Coord3 RTSTransform::operator()(const Coord3 &src) const
        {
            Coord3 src_coords[1];
            Coord3 dst_coords[1];
            src_coords[0] = src;
            transformCoord(1, &src_coords[0], &dst_coords[0]);
            Coord3 dst = dst_coords[0];
            return dst;
        }

        Pose RTSTransform::operator()(const Pose &src) const
        {
            Pose src_poses[1];
            Pose dst_poses[1];
            src_poses[0] = src;
            transformPose(1, &src_poses[0], &dst_poses[0]);
            Pose dst = dst_poses[0];
            return dst;
        }

        void RTSTransform::transformCoord(size_t num_coord, const Coord3 *src, Coord3 *dst) const
        {
            // 数组转换为Eigen矩阵
            const Eigen::Matrix3d r_mat = Eigen::Map<
                const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&R[0]);
            const Eigen::Vector3d d_vec = Eigen::Map<const Eigen::Vector3d>(&T[0]);

            Eigen::Vector3d src_vec, dst_vec;

            // 迭代转换点坐标
            for (int i = 0; i < num_coord; ++i)
            {
                src_vec = Eigen::Vector3d(src[i].x(), src[i].y(), src[i].z());
                dst_vec = S * r_mat * src_vec + d_vec;
                dst[i].set(dst_vec.x(), dst_vec.y(), dst_vec.z());
            }
        }

        void RTSTransform::transformPose(size_t num_pose, const Pose *src, Pose *dst) const
        {
            // 数组转换为eigen矩阵
            const Eigen::Matrix3d r_mat = Eigen::Map<
                const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&R[0]);

            const Eigen::Vector3d d_vec = Eigen::Map<const Eigen::Vector3d>(&T[0]);

            Eigen::Matrix3d src_pose_r_mat, dst_pose_r_mat;
            Eigen::Vector3d src_vec, dst_vec;

            double src_rot_arr[9];
            double dst_rot_arr[9];

            // 迭代批量转换
            for (int i = 0; i < num_pose; ++i)
            {
                const Coord3 &src_center = src[i].center();

                memcpy(src_rot_arr, src[i].rotation(), 9 * sizeof(double));
                src_pose_r_mat = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(src_rot_arr);

                src_vec = Eigen::Vector3d(src_center.x(), src_center.y(), src_center.z());
                dst_vec = S * r_mat * src_vec + d_vec;
                dst_pose_r_mat = src_pose_r_mat * r_mat.transpose();

                Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(dst_rot_arr, 3, 3) =
                    dst_pose_r_mat;

                dst[i].setCenter(Coord3(dst_vec.x(), dst_vec.y(), dst_vec.z()));
                dst[i].setRotation(&dst_rot_arr[0]);
            }
        }

        bool RTSTransform::calcRTS(int num_coord, const Coord3 *src, const Coord3 *dst, double *residuals)
        {
            //点集数量少于3组，无法计算
            if (num_coord < 3)
                return false;

            std::vector<Eigen::Vector3d> vec_src, vec_dst;
            vec_src.resize(num_coord);
            vec_dst.resize(num_coord);
            for (int i = 0; i < num_coord; i++)
            {
                vec_src[i] << src[i].x(), src[i].y(), src[i].z();
                vec_dst[i] << dst[i].x(), dst[i].y(), dst[i].z();
            }

            const Eigen::Matrix3d x1 = Eigen::Map<Eigen::Matrix3d>(vec_src[0].data(), 3, vec_src.size());
            const Eigen::Matrix3d x2 = Eigen::Map<Eigen::Matrix3d>(vec_dst[0].data(), 3, vec_dst.size());
            double _S;
            Eigen::Vector3d _t;
            Eigen::Matrix3d _R;

            if (x1.cols() < 3 || x2.cols() < 3)
            {
                return false;
            }

            assert(3 == x1.rows());
            assert(3 <= x1.cols());
            assert(x1.rows() == x2.rows());
            assert(x1.cols() == x2.cols());

            // Get the transformation via Umeyama's least squares algorithm. This returns
            // a matrix of the form:
            // [ s * R t]
            // [ 0 1]
            // from which we can extract the scale, rotation, and translation.
            const Eigen::Matrix4d transform = Eigen::umeyama(x1, x2, true);

            // Check critical cases
            _R = transform.topLeftCorner<3, 3>();
            if (_R.determinant() < 0)
            {
                return false;
            }
            _S = pow(_R.determinant(), 1.0 / 3.0);
            // Check for degenerate case (if all points have the same value...)
            if (_S < std::numeric_limits<double>::epsilon())
            {
                return false;
            }

            // Extract transformation parameters
            _S = pow(_R.determinant(), 1.0 / 3.0);
            _R /= _S;
            _t = transform.topRightCorner<3, 1>();

            S = _S;
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R, 3, 3) = _R;
            T[0] = _t[0];
            T[1] = _t[1];
            T[2] = _t[2];

            if (residuals)
            {
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

        RTSTransform RTSTransform::inverse() const
        {
            const Eigen::Vector3d d_vec = Eigen::Map<const Eigen::Vector3d>(&T[0]);
            const Eigen::Matrix3d r_mat = Eigen::Map<
                const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&R[0]);
            Eigen::Matrix3d r_mat_tp = r_mat.transpose();

            double s_n = 1.0 / S;
            Eigen::Vector3d d_vec_n = -1.0 * s_n * r_mat_tp * d_vec;

            RTSTransform tsf;
            tsf.S = s_n;
            tsf.R[0] = r_mat_tp(0, 0);
            tsf.R[1] = r_mat_tp(0, 1);
            tsf.R[2] = r_mat_tp(0, 2);
            tsf.R[3] = r_mat_tp(1, 0);
            tsf.R[4] = r_mat_tp(1, 1);
            tsf.R[5] = r_mat_tp(1, 2);
            tsf.R[6] = r_mat_tp(2, 0);
            tsf.R[7] = r_mat_tp(2, 1);
            tsf.R[8] = r_mat_tp(2, 2);
            tsf.T[0] = d_vec_n[0];
            tsf.T[1] = d_vec_n[1];
            tsf.T[2] = d_vec_n[2];
            return tsf;
        }

        void RTSTransform::printTransform() const
        {
            std::cout
                << "RTS transformation:\n"
                << std::fixed << std::setprecision(9)
                << " -- scale:       " << S << "\n"
                << " -- rotation:    ["
                << R[0] << ", " << R[1] << ", " << R[2] << ", "
                << R[3] << ", " << R[4] << ", " << R[5] << ", "
                << R[6] << ", " << R[7] << ", " << R[8] << "]\n"
                << " -- translation: ["
                << T[0] << ", " << T[1] << ", " << T[2] << "]"
                << std::endl;
        }

    } // namespace SRS
} // namespace TJH
