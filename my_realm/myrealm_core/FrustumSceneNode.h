#ifndef MYREALM_FRUSTUMSCENENODE_H
#define MYREALM_FRUSTUMSCENENODE_H

#include "MyREALM_Core_Exports.h"
#include <osg/Geode>
#include <realm_core/camera.h>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace MyREALM
{

	struct Frustum
	{
		/// Camera centre and the 4 points that define the image plane
		Eigen::Vector3d cones[5];

		/// Near clip plane distance
		double z_near;

		/// Far clip plane distance
		double z_far;

		/// Support points
		std::vector<Eigen::Vector3d> points;

		/**
		* @brief Default constructor
		*/
		Frustum();

		/**
		* @brief Build a frustum from the image size, camera intrinsic and pose
		* @param w Width of image plane
		* @param h Height of image plane
		* @param K Intrinsic matrix
		* @param R Extrinsic rotation matrix
		* @param C Center of the camera (optical center)
		*/
		Frustum
		(
			const int w,
			const int h,
			const Eigen::Matrix<double,3,3>& K,
			const Eigen::Matrix<double, 3, 3>& R,
			const Eigen::Vector3d& C
		);

		/**
		* @brief Build a frustum from the image size, camera intrinsic and pose
		* @param w Width of image plane
		* @param h Height of image plane
		* @param K Intrinsic matrix
		* @param R Extrinsic rotation matrix
		* @param C Center of the camera (optical center)
		* @param Specify a far plane
		*/
		Frustum
		(
			const int w,
			const int h,
			const Eigen::Matrix<double, 3, 3>& K,
			const Eigen::Matrix<double, 3, 3>& R,
			const Eigen::Vector3d& C,
			const double zFar
		);

		
		/**
		* @brief Return the supporting frustum points
		* @return Supporting frustum points
		* @note 5 points for infinite frustum
		* @note 8 points for truncated frustum
		*/
		const std::vector<Eigen::Vector3d>& frustum_points() const;


	}; // struct Frustum

	osg::ref_ptr<osg::Geode> createFrustumNode(
		const realm::camera::Pinhole::ConstPtr& cam, double zFar, 
		const osg::Vec4& color = osg::Vec4(1., 1., 1., 1.));

}


#endif // !MYREALM_FRUSTUMSCENENODE_H
