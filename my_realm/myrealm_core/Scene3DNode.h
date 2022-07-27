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
#include <osg/MatrixTransform>
#include <osg/Callback>
#include <osg/Image>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

#include "MyPublisher.h"
#include "MySubscriber.h"
#include "NodeParas.h"
#include "TrajSceneNode.h"
#include "SparseSceneNode.h"
#include "MeshSceneNode.h"
#include "FrustumSceneNode.h"
#include "TileMesh.h"

using namespace realm;

namespace MyREALM
{

	using FollowFrameViewCLB = std::function<void(const osg::Matrix& view_mat,
		int width, int height, double focal, double cx, double cy)>;

	class MyREALM_Core_API Scene3DNode : public OpenThreads::Thread
	{
	public:
		Scene3DNode(const NodeParas& node_paras);
		~Scene3DNode();

		osg::Node* visualTrajNode();

		osg::Node* gnssTrajNode();

		osg::Node* sparseNode();

		osg::Node* denseNode();

		//osg::Group* facesNode();

		osg::Node* meshNode();

		osg::Node* visualPoseFrustumNode();

		osg::Node* gnssPoseFrustumNode();

		osg::Node* arNode();

		virtual int cancel();

		virtual void run();


		void bindFollowFrameViewCLB(FollowFrameViewCLB clb);

		void writeMeshSrs(const std::string& filename);

	private:

		OpenThreads::Mutex _mutex;
		bool _done;
		bool _dirty;

		// camera info
		std::string _id_camera;

		// ros handle
		//ros::NodeHandle _nh;
		NodeParas _node_paras;

		std::shared_ptr<MySubscriber> _sub_output_dir;
		std::shared_ptr<MySubscriber> _sub_input_frame;
		std::shared_ptr<MySubscriber> _sub_visual_pose;
		std::shared_ptr<MySubscriber> _sub_gnss_pose;
		std::shared_ptr<MySubscriber> _sub_gnss_traj;
		std::shared_ptr<MySubscriber> _sub_visual_traj;
		std::shared_ptr<MySubscriber> _sub_gnss_base;

		std::shared_ptr<MySubscriber> _sub_sparse;
		std::shared_ptr<MySubscriber> _sub_dense;
		std::shared_ptr<MySubscriber> _sub_faces;
		std::shared_ptr<MySubscriber> _sub_elevation_ortho;

		// working paths
		std::string _path_working_directory;
		std::string _path_profile;
		std::string _path_output;

		// working directories
		std::string _dir_date_time;

		std::string _mesh_output_directory;

		// filename of settings
		std::string _file_settings_stage;
		std::string _file_settings_method;
		std::string _file_settings_camera;
		std::string _file_settings_imu;

		std::string _id_node;
		// topics
		std::string _topic_prefix;
		std::string _topic_frame_in;
		std::string _topic_visual_pose_in;
		std::string _topic_visual_traj_in;
		std::string _topic_gnss_pose_in;
		std::string _topic_gnss_traj_in;
		std::string _topic_gnss_base_in;
		std::string _topic_sparse_in;
		std::string _topic_dense_in;
		std::string _topic_faces_in;
		std::string _topic_update_elevation_ortho_in;
		std::string _sub_output_dir_in;

		cv::Vec3d _gnss_base_wgs84;
		cv::Vec3d _gnss_base_utm;
		std::string _gnss_base_utm_name;
		int _gnss_base_utm_epsg;
		bool _gnss_base_inited;

		CameraSettings::Ptr _settings_camera;

		osg::ref_ptr<osg::Geode> m_visualTraj;
		osg::ref_ptr<osg::Geode> m_gnssTraj;
		osg::ref_ptr<osg::Geode> m_sparse;
		osg::ref_ptr<osg::Geode> m_dense;
		osg::ref_ptr<osg::Group> m_mesh;
		//osg::ref_ptr<osg::Group> m_faces;
		osg::ref_ptr<osg::MatrixTransform> m_gnssPoseFrustum;
		osg::ref_ptr<osg::MatrixTransform> m_visualPoseFrustum;
		osg::ref_ptr<osg::MatrixTransform> m_arNode;
		osg::ref_ptr<osg::Image> m_arImg;

		osg::ref_ptr<TrajDrawCallback> m_visualTrajCLB;
		osg::ref_ptr<TrajDrawCallback> m_gnssTrajCLB;
		osg::ref_ptr<SparseDrawCallback> m_sparseCLB;
		osg::ref_ptr<SparseDrawCallback> m_denseCLB;
		osg::ref_ptr<MeshNodeCallback> m_meshCLB;
		osg::ref_ptr<ArImageUpdateCallback> m_arCLB;

		TrajReceiverThread* m_visualTrajThread;
		TrajReceiverThread* m_gnssTrajThread;
		SparseReceiverThread* m_sparseThread;
		SparseReceiverThread* m_denseThread;
		MeshReceiverThread* m_meshThread;
		ArImageReceiverThread* m_arThread;
		TileMesh::Ptr m_tile_mesh;

		FollowFrameViewCLB m_followFrameViewCLB;
		int m_update_mesh_count;

		// Initialization
		void readParams();
		void readStageSettings();
		void setPaths();

		void subOutputPath(const std::string& path);

		// ros communication functions
		void subFrame(const realm::Frame::Ptr& frame);

		void subGnssPose(const cv::Mat& pose, uint8_t zone, char band);

		void subVisualPose(const cv::Mat& pose, uint8_t zone, char band);

		void subVisualTraj(const std::vector<cv::Vec3d>& traj);

		void subGnssTraj(const std::vector<cv::Vec3d>& traj);

		void subGnssBase(const cv::Vec3d& gnss_base);

		void subSparse(const realm::PointCloud::Ptr& sparse_cloud);

		void subDense(const realm::PointCloud::Ptr& sparse_cloud);

		void subMesh(const realm::Mesh::Ptr& mesh);

		void subUpdateElevationOrtho(const realm::CvGridMap& map, uint8_t zone, char band);

	};
}

#endif  // !MYREALM_SCENE3DNODE_H
