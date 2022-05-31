#ifndef MYREALM_FRAMEVIEWNODE_H
#define MYREALM_FRAMEVIEWNODE_H


#include "MyREALM_Core_Exports.h"
#include <mutex>
#include <unordered_map>
#include <string>

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

#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

#include "MyPublisher.h"
#include "MySubscriber.h"
#include "NodeParas.h"



using namespace realm;

namespace MyREALM
{
	class MyREALM_Core_API FrameViewNode : public OpenThreads::Thread
	{
	public:
		FrameViewNode(const NodeParas& node_paras);
		~FrameViewNode();

		void bindDisplayTrackedImageFunc(SubImageFun func);

		virtual int cancel();

		virtual void run();

	private:

		OpenThreads::Mutex _mutex;
		bool _done;
		bool _dirty;


		// camera info
		std::string _id_camera;

		// ros handle
		//ros::NodeHandle _nh;
		NodeParas _node_paras;

		// ros communication handles
		std::shared_ptr<MySubscriber> _sub_tracked;

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

		std::string _id_node;
		// topics
		std::string _topic_prefix;
		std::string _topic_tracked;


		CameraSettings::Ptr _settings_camera;

		SubImageFun m_displayTrackedImageFunc;

		// Initialization
		void readParams();
		void readStageSettings();
		void setPaths();


		void subImage(const cv::Mat& image);



	};
}

#endif  // !MYREALM_FRAMEVIEWNODE_H
