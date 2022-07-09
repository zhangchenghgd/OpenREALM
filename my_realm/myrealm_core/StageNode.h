#ifndef PROJECT_STAGE_NODE_H
#define PROJECT_STAGE_NODE_H

#include <mutex>
#include <unordered_map>
#include <string>

#include "MyREALM_Core_Exports.h"

#include <realm_core/camera_settings.h>
#include <realm_core/structs.h>
#include <realm_core/camera_settings_factory.h>
#include <realm_vslam_base/visual_slam_settings_factory.h>
#include <realm_densifier_base/densifier_settings_factory.h>
#include <realm_io/utilities.h>

#include <realm_stages/stage_settings_factory.h>
#include <realm_stages/pose_estimation.h>
#include <realm_stages/densification.h>
#include <realm_stages/surface_generation.h>
#include <realm_stages/ortho_rectification.h>
#include <realm_stages/mosaicing.h>
#include <realm_stages/tileing.h>
#include <Eigen/Core>

#include "MyPublisher.h"
#include "MySubscriber.h"
#include "NodeParas.h"

#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

using namespace realm;

namespace MyREALM
{

	class MyREALM_Core_API StageNode : public OpenThreads::Thread
	{
	public:
		StageNode(const NodeParas& node_paras);
		~StageNode();
		void spin();

		virtual int cancel();
		virtual void run();

		void pause();
		void resume();

		std::string outputDirDatetime() const;

	private:
		OpenThreads::Mutex _mutex;
		bool _pause;

		// Master stage has several privileges,
		// e.g. creating output folder, ...
		bool _is_master_stage;

		// Set to true, to shut node down
		std::mutex _mutex_do_shutdown;
		bool _do_shutdown;
		bool _is_stage_running;

		// type of stage
		std::string _type_stage;
		std::string _type_method;

		// camera info
		std::string _id_camera;

		NodeParas _node_paras;

		// ros communication handles
		std::shared_ptr<MySubscriber> _sub_input_frame;
		std::shared_ptr<MySubscriber> _sub_input_imu;
		std::shared_ptr<MySubscriber> _sub_output_dir;
		std::unordered_map<std::string, MyPublisher*> _publisher;

		// chosen profile related to all settings set
		std::string _profile;

		// chosen method related to the specific framework implementation
		std::string _method;

		// working paths
		std::string _path_working_directory;
		std::string _path_profile;
		std::string _path_output;

		// working directories
		std::string _dir_date_time;

		// filename of settings
		std::string _file_settings_stage;
		std::string _file_settings_method;
		std::string _file_settings_camera;
		std::string _file_settings_imu;

		// topics
		std::string _topic_prefix;
		std::string _topic_frame_in;
		std::string _topic_frame_out;
		std::string _topic_imu_in;

		// trajectories
		std::unordered_map<std::string, std::vector<cv::Vec3d>> _trajectories;

		bool _gnss_base_initialized;
		cv::Vec3d _gnss_base;

		// Settings of the stage
		StageSettings::Ptr _settings_stage;

		CameraSettings::Ptr _settings_camera;

		// Handle for stage
		StageBase::Ptr _stage;

		// Initialization
		void readParams();
		void readStageSettings();
		void setPaths();
		void createStagePoseEstimation();
		void createStageDensification();
		void createStageSurfaceGeneration();
		void createStageOrthoRectification();
		void createStageMosaicing();
		void createStageTileing();
		void linkStageTransport();

		// ros communication functions
		void subFrame(const realm::Frame::Ptr& frame);
		void subImu(const VisualSlamIF::ImuData& imu);
		void subOutputPath(const std::string& path);

		// stage callbacks
		void pubFrame(const Frame::Ptr& frame, const std::string& topic);
		void pubPose(const cv::Mat& pose, uint8_t zone, char band, const std::string& topic);
		void pubPointCloud(const PointCloud::Ptr& sparse_cloud, const std::string& topic);
		void pubImage(const cv::Mat& img, const std::string& topic);
		void pubDepthMap(const cv::Mat& img, const std::string& topic);
		void pubMesh(const realm::Mesh::Ptr& mesh, const std::string& topic);

		// master publish
		void pubTrajectory(const std::vector<cv::Vec3d>& traj, const std::string& topic);

		/*!
		 * @brief Publisher for CvGridMaps as GroundImages. Should either contain one or two layers. The first layer
		 *        represents the visual informations and the second (optional) the mask or valid elements of the visual info.
		 * @param map Either 1- or 2-layer grid map containing visual informations (and valid elements of visual img)
		 * @param zone UTM zone of the published CvGridMap
		 * @param band UTM band of the published CvGridMap
		 * @param topic Topic is NOT the ros topic, but the topic for REALM to identify which publisher should be triggered.
		 */
		void pubCvGridMap(const CvGridMap& map, uint8_t zone, char band, const std::string& topic);

		bool srvFinish();
		bool srvStop();
		bool srvResume();
		bool srvReset();
		bool srvChangeParam(const std::string& para_name, const std::string& para_val);
	};

}

#endif //PROJECT_STAGE_NODE_H
