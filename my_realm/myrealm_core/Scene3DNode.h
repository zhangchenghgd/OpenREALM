#ifndef MYREALM_SCENE3DNODE_H
#define MYREALM_SCENE3DNODE_H


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

#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

#include "MyPublisher.h"
#include "MySubscriber.h"
#include "NodeParas.h"
#include "TrajSceneNode.h"
#include "SparseSceneNode.h"
#include "FacesSceneNode.h"


using namespace realm;

namespace MyREALM
{
	class MyREALM_Core_API Scene3DNode : public OpenThreads::Thread
	{
	public:
		Scene3DNode(const NodeParas& node_paras);
		~Scene3DNode();

		osg::Geode* visualTrajNode();

		osg::Geode* gnssTrajNode();

		osg::Geode* sparseNode();

		osg::Geode* facesNode();

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
		std::shared_ptr<MySubscriber> _sub_input_frame;
		std::shared_ptr<MySubscriber> _sub_input_pose;
		std::shared_ptr<MySubscriber> _sub_gnss_traj;
		std::shared_ptr<MySubscriber> _sub_visual_traj;
		std::shared_ptr<MySubscriber> _sub_gnss_base; 

		std::shared_ptr<MySubscriber> _sub_sparse;
		std::shared_ptr<MySubscriber> _sub_faces;
		
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
		std::string _topic_frame_in;
		std::string _topic_visual_traj_in;
		std::string _topic_gnss_traj_in;
		std::string _topic_gnss_base_in;
		std::string _topic_sparse_in;
		std::string _topic_faces_in;

		cv::Vec3d _gnss_base_wgs84;
		cv::Vec3d _gnss_base_utm;

		CameraSettings::Ptr _settings_camera;

		osg::ref_ptr<osg::Geode> m_visualTraj;
		osg::ref_ptr<osg::Geode> m_gnssTraj;
		osg::ref_ptr<osg::Geode> m_sparse;
		osg::ref_ptr<osg::Geode> m_faces;

		osg::ref_ptr<TrajDrawCallback> m_visualTrajCLB;
		osg::ref_ptr<TrajDrawCallback> m_gnssTrajCLB;
		osg::ref_ptr<SparseDrawCallback> m_sparseCLB;
		osg::ref_ptr<FacesDrawCallback> m_facesCLB;

		TrajReceiverThread* m_visualTrajThread;
		TrajReceiverThread* m_gnssTrajThread;
		SparseReceiverThread* m_sparseThread;
		FacesReceiverThread* m_facesThread;

		// Initialization
		void readParams();
		void readStageSettings();
		void setPaths();

		// ros communication functions
		void subFrame(const realm::Frame::Ptr& frame);

		void subPose(const cv::Mat& pose);

		void subVisualTraj(const std::vector<cv::Vec3d>& traj);

		void subGnssTraj(const std::vector<cv::Vec3d>& traj);

		void subGnssBase(const cv::Vec3d& gnss_base);

		void subSparse(const realm::PointCloud::Ptr& sparse_cloud);

		void subFaces(const  std::vector<realm::Face>& faces);

	};
}

#endif  // !MYREALM_SCENE3DNODE_H
