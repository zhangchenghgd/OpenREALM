#include "MyPublisher.h"

namespace MyREALM
{
	MyPublisher::MyPublisher()
	{

	}

	MyPublisher::~MyPublisher()
	{

	}

	void MyPublisher::pubFrame(const realm::Frame::Ptr& frame)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subFrame(frame);
		}
	}

	void MyPublisher::pubPointCloud(const realm::PointCloud::Ptr& sparse_cloud)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subSparse(sparse_cloud);
		}
	}

	void MyPublisher::pubImage(const cv::Mat& img)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subImage(img);
		}
	}

	void MyPublisher::pubDepthMap(const cv::Mat& depthMap)
	{
	}

	void MyPublisher::pubPose(const cv::Mat& pose, uint8_t zone, char band)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subPose(pose, zone, band);
		}
	}

	void MyPublisher::pubMesh(const realm::Mesh::Ptr& mesh)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subFaces(mesh);
		}
	}


	void MyPublisher::pubCvGridMap(const realm::CvGridMap& map, uint8_t zone, char band)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subCvGridMap(map, zone, band);
		}
	}

	void MyPublisher::pubOutputDir(const std::string& dir)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subOutDir(dir);
		}
	}

	void MyPublisher::pubGnssBase(const cv::Vec3d& gnss_base)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subGnssBase(gnss_base);
		}
	}

	void MyPublisher::pubTrajectory(const std::vector<cv::Vec3d>& traj)
	{
		for (MySubscribers::iterator it = m_subscribers.begin();
			it != m_subscribers.end(); ++it)
		{
			it->second->subTrajectory(traj);
		}
	}


	std::shared_ptr<MySubscriber> MyPublisher::registSubscriber(const std::string& name)
	{
		std::shared_ptr<MySubscriber> sub = std::make_shared<MySubscriber>();
		m_subscribers[name] = sub;
		return sub;
	}

	void MyPublisher::unregistSubscriber(const std::string& name)
	{
		MySubscribers::iterator it = m_subscribers.find(name);
		if (it != m_subscribers.end())
		{
			m_subscribers.erase(name);
		}
		m_mutex.unlock();
	}

	MySubscriber* MyPublisher::getSubscriber(const std::string& name)
	{
		MySubscriber* sub = nullptr;
		MySubscribers::iterator it = m_subscribers.find(name);
		if (it != m_subscribers.end())
		{
			sub = it->second.get();
		}
		return sub;
	}

	int MyPublisher::getNumSubscribers()
	{
		return m_subscribers.size();
	}

}
