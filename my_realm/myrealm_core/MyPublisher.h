#ifndef MYREALM_PUBLISHER_H
#define MYREALM_PUBLISHER_H

#include "MyREALM_Core_Exports.h"
#include "realm_core/frame.h"
#include "realm_core/point_cloud.h"
#include "realm_core/structs.h"
#include "realm_core/cv_grid_map.h"
#include "MySubscriber.h"
#include <mutex>
#include <thread>
#include <unordered_map>
#include <memory>

namespace MyREALM
{
	class MyREALM_Core_API MyPublisher
	{
	public:
		MyPublisher();
		~MyPublisher();

		void pubFrame(const realm::Frame::Ptr& frame);

		void pubPointCloud(const realm::PointCloud::Ptr& sparse_cloud);

		void pubImage(const cv::Mat& img);

		void pubDepthMap(const cv::Mat& depthMap);

		void pubPose(const cv::Mat& pose, uint8_t zone, char band);

		void pubMesh(const realm::Mesh::Ptr& mesh);

		void pubCvGridMap(const realm::CvGridMap& map, uint8_t zone, char band);

		void pubOutputDir(const std::string& dir);

		void pubGnssBase(const cv::Vec3d& gnss_base);

		void pubTrajectory(const std::vector<cv::Vec3d>& traj);

		std::shared_ptr<MySubscriber> registSubscriber(const std::string& name);

		void unregistSubscriber(const std::string& name);

		MySubscriber* getSubscriber(const std::string& name);

		int getNumSubscribers();

	private:
		std::mutex m_mutex;
		MySubscribers m_subscribers;
	};

	typedef std::unordered_map<std::string, std::shared_ptr<MyPublisher>> MyPublishers;
}

#endif // !MYREALM_PUBLISHER_H