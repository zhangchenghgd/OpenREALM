#ifndef MYREALM_SUBSCRIBER_H
#define MYREALM_SUBSCRIBER_H


#include "MyREALM_Core_Exports.h"
#include "realm_core/frame.h"
#include "realm_core/structs.h"
#include "realm_vslam_base/visual_slam_IF.h"
#include <unordered_map>
#include <memory>
#include <functional>

namespace MyREALM
{
	using SubFrameFun = std::function<void(const realm::Frame::Ptr& frame)>;
	using SubPoseFun = std::function<void(const cv::Mat& pose, uint8_t zone, char band)>;
	using SubImageFun = std::function<void(const cv::Mat& img)>;
	using SubImuFun = std::function<void(const realm::VisualSlamIF::ImuData& imu)>;
	using SubOutDirFun = std::function<void(const std::string& dir)>;
	using SubTrajFun = std::function<void(const std::vector<cv::Vec3d>& traj)>;
	using SubGnssBaseFun = std::function<void(const cv::Vec3d& gnss_base)>;
	using SubSparseFun = std::function<void(const realm::PointCloud::Ptr& sparse_cloud)>;
	/*using SubFacesFun = std::function<void(const std::vector<realm::Face>& faces)>;*/
	using SubMeshFun = std::function<void(const realm::Mesh::Ptr& mesh)>;
	using SubCvGridMapFun = std::function<void(const realm::CvGridMap& map, uint8_t zone, char band)>;

	class MyREALM_Core_API MySubscriber
	{
	public:
		MySubscriber();
		~MySubscriber();

		void subFrame(const realm::Frame::Ptr& frame);

		void subImage(const cv::Mat& img);

		void subPose(const cv::Mat& pose, uint8_t zone, char band);

		void subOutDir(const std::string& dir);

		void subImu(const realm::VisualSlamIF::ImuData& imu);

		void subTrajectory(const std::vector<cv::Vec3d>& traj);

		void subGnssBase(const cv::Vec3d& gnss_base);

		void subSparse(const realm::PointCloud::Ptr& sparse_cloud);

		void subFaces(const realm::Mesh::Ptr& mesh);

		void subCvGridMap(const realm::CvGridMap& ortho, uint8_t zone, char band);

		void bindSubFrameFunc(SubFrameFun func);

		void bindSubPoseFunc(SubPoseFun func);

		void bindSubOutDirFunc(SubOutDirFun func);

		void bindSubImuFunc(SubImuFun func);

		void bindSubTrajFunc(SubTrajFun func);

		void bindSubGnssBaseFunc(SubGnssBaseFun func);

		void bindSubSparseFunc(SubSparseFun func);

		void bindSubMeshFunc(SubMeshFun func);

		void bindSubImageFunc(SubImageFun func);

		void bindCvGridMapFunc(SubCvGridMapFun func);


	private:
		SubFrameFun m_SubFrameFunc;
		SubPoseFun m_SubPoseFunc;
		SubOutDirFun m_SubOutDirFunc;
		SubImuFun m_SubImuFunc;
		SubTrajFun m_SubTrajFun;
		SubGnssBaseFun m_SubGnssBaseFun;
		SubSparseFun m_SubSparseFun;
		SubMeshFun m_SubMeshFunc;
		SubImageFun m_SubImageFunc;
		SubCvGridMapFun m_CvGridMapFunc;

	};

	typedef std::unordered_map<std::string, std::shared_ptr<MySubscriber>> MySubscribers;

}

#endif // !MYREALM_SUBSCRIBER_H