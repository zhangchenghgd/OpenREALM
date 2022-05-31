#include "MySubscriber.h"

namespace MyREALM
{
	MySubscriber::MySubscriber() :
		m_SubFrameFunc(NULL),
		m_SubPoseFunc(NULL),
		m_SubOutDirFunc(NULL),
		m_SubImuFunc(NULL),
		m_SubTrajFun(NULL),
		m_SubGnssBaseFun(NULL),
		m_SubSparseFun(NULL),
		m_SubFacesFunc(NULL),
		m_SubImageFunc(NULL)
	{

	}

	MySubscriber::~MySubscriber()
	{

	}
	void MySubscriber::subFrame(const realm::Frame::Ptr& frame)
	{
		if (m_SubFrameFunc)
		{
			m_SubFrameFunc(frame);
		}

	}

	void MySubscriber::subImage(const cv::Mat& img)
	{
		if (m_SubImageFunc)
		{
			m_SubImageFunc(img);
		}
	}

	void MySubscriber::subPose(const cv::Mat& pose, uint8_t zone, char band)
	{
		if (m_SubPoseFunc)
		{
			m_SubPoseFunc(pose, zone, band);
		}
	}

	void MySubscriber::subOutDir(const std::string& dir)
	{
		if (m_SubOutDirFunc)
		{
			m_SubOutDirFunc(dir);
		}
	}

	void MySubscriber::subImu(const realm::VisualSlamIF::ImuData& imu)
	{
		if (m_SubImuFunc)
		{
			m_SubImuFunc(imu);
		}
	}

	void MySubscriber::subTrajectory(const std::vector<cv::Vec3d>& traj)
	{
		if (m_SubTrajFun)
		{
			m_SubTrajFun(traj);
		}
	}

	void MySubscriber::subGnssBase(const cv::Vec3d& gnss_base)
	{
		if (m_SubGnssBaseFun)
		{
			m_SubGnssBaseFun(gnss_base);
		}
	}

	void MySubscriber::subSparse(const realm::PointCloud::Ptr& sparse_cloud)
	{
		if (m_SubSparseFun)
		{
			m_SubSparseFun(sparse_cloud);
		}
	}

	void MySubscriber::subFaces(const std::vector<realm::Face>& faces)
	{
		if (m_SubFacesFunc)
		{
			m_SubFacesFunc(faces);
		}
	}

	void MySubscriber::bindSubFrameFunc(SubFrameFun func)
	{
		m_SubFrameFunc = func;
	}

	void MySubscriber::bindSubPoseFunc(SubPoseFun func)
	{
		m_SubPoseFunc = func;
	}
	void MySubscriber::bindSubImuFunc(SubImuFun func)
	{
		m_SubImuFunc = func;
	}

	void MySubscriber::bindSubTrajFunc(SubTrajFun func)
	{
		m_SubTrajFun = func;
	}

	void MySubscriber::bindSubGnssBaseFunc(SubGnssBaseFun func)
	{
		m_SubGnssBaseFun = func;
	}

	void MySubscriber::bindSubSparseFunc(SubSparseFun func)
	{
		m_SubSparseFun = func;
	}

	void MySubscriber::bindSubFacesFunc(SubFacesFun func)
	{
		m_SubFacesFunc = func;
	}

	void MySubscriber::bindSubImageFunc(SubImageFun func)
	{
		m_SubImageFunc = func;
	}


	void MySubscriber::bindSubOutDirFunc(SubOutDirFun func)
	{
		m_SubOutDirFunc = func;
	}
}
