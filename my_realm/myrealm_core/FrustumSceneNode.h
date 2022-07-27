﻿#ifndef MYREALM_FRUSTUMSCENENODE_H
#define MYREALM_FRUSTUMSCENENODE_H

#include "MyREALM_Core_Exports.h"
#include <deque>
#include <osg/Geode>
#include <osg/Image>
#include <osg/Callback>
#include <osg/StateSet>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <realm_core/camera.h>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace MyREALM
{

	class ArImageUpdateCallback;

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

	osg::ref_ptr<osg::Geode> generateARImageNode(
		const realm::camera::Pinhole::ConstPtr& cam, double zFar,
		ArImageUpdateCallback* clb);

	void removeArImageUpdateCallback(osg::Geode* geode, ArImageUpdateCallback* clb);


	/**
	 * @brief 轨迹线接收集合顶点线程.
	 */
	class ArImageReceiverThread : public OpenThreads::Thread
	{
	public:
		ArImageReceiverThread();
		~ArImageReceiverThread();

		virtual int cancel();
		virtual void run();

		void setImage(int w, int h,
			GLenum pixelFormat, GLenum type,
			unsigned char* data);

		bool getImage(osg::ref_ptr<osg::Image>& img);

		void clear();

		inline bool isDirty() const { return _img_que.size() > 0; }

	protected:
		OpenThreads::Mutex _mutex;
		std::deque<osg::ref_ptr<osg::Image>> _img_que;
		size_t _max_que_size;
		bool _done;
	};


	class ArImageUpdateCallback :public osg::StateAttributeCallback
	{
	public:
		ArImageUpdateCallback(ArImageReceiverThread* p_thread);
		~ArImageUpdateCallback();

		virtual void operator () (osg::StateAttribute*, osg::NodeVisitor*) override;

	protected:
		ArImageReceiverThread* m_thread;
	};

}


#endif // !MYREALM_FRUSTUMSCENENODE_H
