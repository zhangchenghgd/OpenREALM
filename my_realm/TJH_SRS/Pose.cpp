/**
 * @file Pose.cpp
 * @author ZhangCheng (zhangcheng@whulabs.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-26
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "Pose.h"
#include <memory>
#include <Eigen/Eigen>

namespace TJH
{
    namespace SRS
    {
        Pose::Pose()
        {
            m_center = Coord3();
            // m_rotation 设为单位矩阵
            memset(m_rotation, 0, 9 * sizeof(double));
            m_rotation[0] = m_rotation[4] = m_rotation[8] = 1.0;
        }

        Pose::Pose(const Coord3 &_center, const double *_rotation)
        {
            m_center = _center;
            memcpy(m_rotation, _rotation, 9 * sizeof(double));
        }

        Pose::Pose(double x, double y, double z,
                   double omega, double phi, double kappa)
        {
            m_center.set(x, y, z);
            EulerAngleOPK_To_Matrix(omega, phi, kappa, &m_rotation[0]);
        }

        const Coord3 &Pose::center() const
        {
            return m_center;
        }

        void Pose::setCenter(const Coord3 &_center)
        {
            m_center = _center;
        }

        const double *Pose::rotation() const
        {
            return &m_rotation[0];
        }

        void Pose::setRotation(const double *_rot)
        {
            memcpy(m_rotation, _rot, 9 * sizeof(double));
        }

        void Pose::setRotation(double omega, double phi, double kappa)
        {
            EulerAngleOPK_To_Matrix(omega, phi, kappa, &m_rotation[0]);
        }

        void Pose::getRotationOPK(double *omega, double *phi, double *kappa) const
        {
            Matrix_To_EulerAngleOPK(&m_rotation[0], omega, phi, kappa);
        }

        Pose &Pose::operator=(const Pose &other)
        {
            m_center = other.m_center;
            memcpy(m_rotation, other.m_rotation, 9 * sizeof(double));
            return *this;
        }

        bool Pose::empty() const
        {
            return (m_center.empty() && 
                m_rotation[0] == 1.0 &&
                m_rotation[1] == 0 &&
                m_rotation[2] == 0 &&
                m_rotation[3] == 0 &&
                m_rotation[4] == 1.0 &&
                m_rotation[5] == 0 &&
                m_rotation[6] == 0 &&
                m_rotation[7] == 0 &&
                m_rotation[8] == 1.0 
            );
        }

        void Matrix_To_EulerAngleOPK(
            const double *R,
            double *omega,
            double *phi,
            double *kappa)
        {
            *phi = asin(R[6]) * 180.0 / M_PI;
            *omega = atan2(-R[7], R[8]) * 180.0 / M_PI;
            *kappa = atan2(-R[3], R[0]) * 180.0 / M_PI;
        }

        void EulerAngleOPK_To_Matrix(
            double _omega,
            double _phi,
            double _kappa,
            double *R)
        {

            double omega = _omega * M_PI / 180.0;
            double phi = _phi * M_PI / 180.0;
            double kappa = _kappa * M_PI / 180.0;

            R[0] = cos(phi) * cos(kappa);
            R[1] = cos(omega) * sin(kappa) + sin(omega) * sin(phi) * cos(kappa);
            R[2] = sin(omega) * sin(kappa) - cos(omega) * sin(phi) * cos(kappa);

            R[3] = -cos(phi) * sin(kappa);
            R[4] = cos(omega) * cos(kappa) - sin(omega) * sin(phi) * sin(kappa);
            R[5] = sin(omega) * cos(kappa) + cos(omega) * sin(phi) * sin(kappa);

            R[6] = sin(phi);
            R[7] = -sin(omega) * cos(phi);
            R[8] = cos(omega) * cos(phi);
        }

    } // namespace SRS
} // namespace TJH