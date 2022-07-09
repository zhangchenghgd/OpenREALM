#include "Scene3DNode.h"
#include "MyREALM.h"
#include <sstream>
#include <glog/logging.h>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osgDB/WriteFile>
#include <TJH_SRS/UTM.h>


namespace MyREALM
{

	Scene3DNode::Scene3DNode(const NodeParas& node_paras) :OpenThreads::Thread(),
		_node_paras(node_paras)
	{
		// Read basic launch file inputs
		readParams();

		// Specify stage
		setPaths();

		m_followFrameViewCLB = NULL;

		_gnss_base_wgs84 = cv::Vec3d(0, 0, 0);
		_gnss_base_utm = cv::Vec3d(0, 0, 0);
		m_visualTrajThread = new TrajReceiverThread;
		m_gnssTrajThread = new TrajReceiverThread;
		m_sparseThread = new SparseReceiverThread;
		m_denseThread = new SparseReceiverThread;
		m_meshThread = new FacesReceiverThread;

		m_visualTrajCLB = new TrajDrawCallback(m_visualTrajThread);
		m_gnssTrajCLB = new TrajDrawCallback(m_gnssTrajThread);
		m_sparseCLB = new SparseDrawCallback(m_sparseThread);
		m_denseCLB = new SparseDrawCallback(m_denseThread);
		m_meshCLB = new  FacesDrawCallback(m_meshThread);

		m_visualTraj = createTrajGeode(osg::Vec4(1, 0, 0, 1), 2,
			m_visualTrajCLB);
		m_gnssTraj = createTrajGeode(osg::Vec4(0, 1, 0, 1), 2,
			m_gnssTrajCLB);
		m_sparse = createSparseGeode(osg::Vec4(1, 1, 0, 1), 2,
			m_sparseCLB);
		m_dense = createSparseGeode(osg::Vec4(1, 0, 0, 1), 4,
			m_denseCLB);

		//m_mesh = createFacesGeode(m_meshCLB);
		m_gnssPoseFrustum = new osg::MatrixTransform;
		m_visualPoseFrustum = new osg::MatrixTransform;

		osg::ref_ptr<osg::Vec3Array> mesh_vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> mesh_clr_arr = new osg::Vec4Array;
		osg::ref_ptr<osg::Vec2Array> mesh_texture_arr = new osg::Vec2Array;
		osg::ref_ptr<osg::UIntArray> mesh_face_ary = new osg::UIntArray;

		osg::ref_ptr<osg::Geode> mesh_geode = createMeshGeode(mesh_vec_arr,
			mesh_clr_arr, mesh_texture_arr, mesh_face_ary, nullptr);
		m_mesh = new osg::Group;
		m_mesh->addChild(mesh_geode);

		m_visualTrajThread->startThread();
		m_gnssTrajThread->startThread();
		m_sparseThread->startThread();
		m_denseThread->startThread();
		//m_meshThread->startThread();

		//createFacesNode();

		_topic_prefix = "/realm/" + _id_camera ;

		_topic_frame_in = "/realm/" + _id_camera +"/input";

		_sub_output_dir_in = "/realm/" + _id_camera + "/general/output_dir";

		_topic_visual_pose_in = "/realm/" + _id_camera + "/pose_estimation/pose/visual/utm";
		_topic_gnss_pose_in = "/realm/" + _id_camera + "/pose_estimation/pose/gnss/utm";

		_topic_visual_traj_in= "/realm/"+ _id_camera +"/pose_estimation/pose/visual/traj";

		_topic_gnss_traj_in = "/realm/" + _id_camera + "/pose_estimation/pose/gnss/traj";

		_topic_gnss_base_in = "/realm/" + _id_camera + "/general/gnss_base";


		_topic_sparse_in = "/realm/" + _id_camera + "/pose_estimation/pointcloud";

		
		//ortho_rectification  | densification
		_topic_dense_in = "/realm/" + _id_camera + "/densification/pointcloud";

		_topic_faces_in = "/realm/" + _id_camera + "/mosaicing/mesh";
		_topic_update_ortho_in = "/realm/" + _id_camera + "/mosaicing/update/ortho";
		_topic_update_elevation_in = "/realm/" + _id_camera + "/mosaicing/update/elevation";

		_sub_input_frame = MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_in)
			->registSubscriber("/realm/" + _id_camera + + "/scene3d/sub/frame");

		SubFrameFun subFrameFunc = std::bind(&Scene3DNode::subFrame, this, std::placeholders::_1);
		_sub_input_frame->bindSubFrameFunc(subFrameFunc);

		_sub_output_dir = MyRealmSys::get_instance()
			.getOrCreatePublisher(_sub_output_dir_in)
			->registSubscriber(_sub_output_dir_in + "/sub/output_dir");

		SubOutDirFun subOutDirFunc = std::bind(&Scene3DNode::subOutputPath, this, std::placeholders::_1);
		_sub_output_dir->bindSubOutDirFunc(subOutDirFunc);

		_sub_visual_pose = MyRealmSys::get_instance().getOrCreatePublisher(_topic_visual_pose_in)
			->registSubscriber(_topic_visual_pose_in + "/sub");

		SubPoseFun subVisualPoseFunc = std::bind(&Scene3DNode::subVisualPose, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
		_sub_visual_pose->bindSubPoseFunc(subVisualPoseFunc);

		_sub_gnss_pose = MyRealmSys::get_instance().getOrCreatePublisher(_topic_gnss_pose_in)
			->registSubscriber(_topic_gnss_pose_in + "/sub");

		SubPoseFun subGnssPoseFunc = std::bind(&Scene3DNode::subGnssPose , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		_sub_gnss_pose->bindSubPoseFunc(subGnssPoseFunc);
		
		_sub_visual_traj = MyRealmSys::get_instance().getOrCreatePublisher(_topic_visual_traj_in)
			->registSubscriber(_topic_visual_traj_in+"/sub");

		SubTrajFun subVisualTrajFunc = std::bind(&Scene3DNode::subVisualTraj, this, std::placeholders::_1);
		_sub_visual_traj->bindSubTrajFunc(subVisualTrajFunc);

		_sub_gnss_traj = MyRealmSys::get_instance().getOrCreatePublisher(_topic_gnss_traj_in)
			->registSubscriber(_topic_gnss_traj_in+"/sub");

		SubTrajFun subGnssTrajFunc = std::bind(&Scene3DNode::subGnssTraj , this, std::placeholders::_1);
		_sub_gnss_traj->bindSubTrajFunc(subGnssTrajFunc);

		_sub_gnss_base = MyRealmSys::get_instance().getOrCreatePublisher(_topic_gnss_base_in)
			->registSubscriber(_topic_gnss_base_in + "/sub");

		SubGnssBaseFun subGnssBaseFunc = std::bind(&Scene3DNode::subGnssBase, this, std::placeholders::_1);
		_sub_gnss_base->bindSubGnssBaseFunc(subGnssBaseFunc);

		_sub_sparse = MyRealmSys::get_instance().getOrCreatePublisher(_topic_sparse_in)
			->registSubscriber(_topic_sparse_in + "/sub");

		SubSparseFun subSparseFunc = std::bind(&Scene3DNode::subSparse, this, std::placeholders::_1);
		_sub_sparse->bindSubSparseFunc(subSparseFunc);

		_sub_dense = MyRealmSys::get_instance().getOrCreatePublisher(_topic_dense_in)
			->registSubscriber(_topic_dense_in + "/sub");

		SubSparseFun subDenseFunc = std::bind(&Scene3DNode::subDense, this, std::placeholders::_1);
		_sub_dense->bindSubSparseFunc(subDenseFunc);

		_sub_faces = MyRealmSys::get_instance().getOrCreatePublisher(_topic_faces_in)
			->registSubscriber(_topic_faces_in + "/sub");

		SubMeshFun subFacesFunc = std::bind(&Scene3DNode::subMesh, this, std::placeholders::_1);
		_sub_faces->bindSubMeshFunc(subFacesFunc);
		


		_sub_ortho = MyRealmSys::get_instance().getOrCreatePublisher(_topic_update_ortho_in)
			->registSubscriber(_topic_update_ortho_in + "/sub");

		SubCvGridMapFun subOrthoFunc = std::bind(&Scene3DNode::subOrtho, this, 
			std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		_sub_ortho->bindSubOrthoFunc(subOrthoFunc);

		_sub_elevation = MyRealmSys::get_instance().getOrCreatePublisher(_topic_update_elevation_in)
			->registSubscriber(_topic_update_elevation_in + "/sub");

		SubCvGridMapFun subElevationFunc = std::bind(&Scene3DNode::subElevation, this,
			std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		_sub_elevation->bindSubElevationFunc(subElevationFunc);

	}

	Scene3DNode::~Scene3DNode()
	{
		if(m_visualTraj->getNumChildren()==1) 
			m_visualTraj->getChild(0)->removeUpdateCallback(m_visualTrajCLB);
		if (m_gnssTraj->getNumChildren() == 1) 
			m_gnssTraj->getChild(0)->removeUpdateCallback(m_gnssTrajCLB);
		if (m_sparse->getNumChildren() == 1) 
			m_sparse->getChild(0)->removeUpdateCallback(m_sparseCLB);
		if (m_dense->getNumChildren() == 1)
			m_dense->getChild(0)->removeUpdateCallback(m_denseCLB);
		/*if (m_faces->getNumChildren() == 1)
			m_faces->getChild(0)->removeUpdateCallback(m_facesCLB);*/

		/*m_visualTraj->removeChildren(0, m_visualTraj->getNumChildren());
		m_gnssTraj->removeChildren(0, m_gnssTraj->getNumChildren());
		m_sparse->removeChildren(0, m_sparse->getNumChildren());
		m_mesh->removeChildren(0, m_mesh->getNumChildren());*/
		//m_faces->removeChildren(0, m_faces->getNumChildren());

		if (isRunning())
		{
			this->cancel();
		}

		
		m_visualTrajThread->cancel();
		m_gnssTrajThread->cancel();
		m_sparseThread->cancel();
		m_denseThread->cancel();
		//m_meshThread->cancel();

		delete m_visualTrajThread;
		delete m_gnssTrajThread;
		delete m_sparseThread;
		delete m_denseThread;
		//delete m_meshThread;

		m_visualTrajThread = nullptr;
		m_gnssTrajThread = nullptr;
		m_sparseThread = nullptr;
		m_denseThread = nullptr;
		//m_meshThread = nullptr;

	}


	int Scene3DNode::cancel()
	{
		_done = true;

		if (m_visualTrajThread->isRunning())
		{
			m_visualTrajThread->cancel();
		}
		if (m_gnssTrajThread->isRunning())
		{
			m_gnssTrajThread->cancel();
		}
		if (m_sparseThread->isRunning())
		{
			m_sparseThread->cancel();
		}
		if (m_denseThread->isRunning())
		{
			m_denseThread->cancel();
		}
		/*if (m_meshThread->isRunning())
		{
			m_meshThread->cancel();
		}*/
		/*for (TileFacesNodeMap::iterator tile_it = m_tileFacesNodes.begin();
			tile_it != m_tileFacesNodes.end(); ++tile_it)
		{
			tile_it->second->facesThread()->cancel();
		}*/

		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void Scene3DNode::run()
	{
		_done = false;
		_dirty = true;
		do
		{
			YieldCurrentThread();
		} while (!_done);
	}

	void Scene3DNode::bindFollowFrameViewCLB(FollowFrameViewCLB clb)
	{
		m_followFrameViewCLB = clb;
	}

	osg::Node* Scene3DNode::visualTrajNode()
	{
		return m_visualTraj->asNode();
	}

	osg::Node* Scene3DNode::gnssTrajNode()
	{
		return m_gnssTraj->asNode();
	}

	osg::Node* Scene3DNode::sparseNode()
	{
		return m_sparse->asNode();
	}

	osg::Node* Scene3DNode::denseNode()
	{
		return m_dense->asNode();
	}

	/*osg::Group* Scene3DNode::facesNode()
	{
		return m_faces.get();
	}*/

	osg::Node* Scene3DNode::meshNode()
	{
		return m_mesh->asNode();
	}


	osg::Node* Scene3DNode::visualPoseFrustumNode()
	{
		return m_visualPoseFrustum->asNode();
	}

	osg::Node* Scene3DNode::gnssPoseFrustumNode()
	{
		return m_gnssPoseFrustum->asNode();
	}


	void Scene3DNode::readParams()
	{
		_node_paras.param("config/id", _id_node, std::string("uninitialised"));
		_node_paras.param("config/opt/working_directory", _path_working_directory, std::string("uninitialised"));
		_node_paras.param("config/opt/output_directory", _path_output, std::string("uninitialised"));

		_id_camera = _id_node;

		if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
			throw(std::invalid_argument("Error: Working directory does not exist!"));
	}

	void Scene3DNode::readStageSettings()
	{
	}

	void Scene3DNode::setPaths()
	{
		if (_path_output == "uninitialised")
			_path_output = _path_working_directory + "/output";
	}


	void Scene3DNode::subOutputPath(const std::string& p_dir_date_time)
	{
		// check if output directory has changed
		if (_dir_date_time != p_dir_date_time)
		{
			// Note: master privilege is not to create folder, but to set the name of the folder
			_dir_date_time = p_dir_date_time;
			std::string output_datetime_dirname = _path_output + "/" + _dir_date_time;
			if (!io::dirExists(output_datetime_dirname))
				io::createDir(output_datetime_dirname);
			
			_mesh_output_directory = output_datetime_dirname + "/mesh";
			if (!io::dirExists(_mesh_output_directory))
				io::createDir(_mesh_output_directory);

			// Debug info
			LOG(INFO) << "Scene3DNode: Received output directory, set to:\n\t" << _path_output + "/" + _dir_date_time;
		}
	}

	void Scene3DNode::subFrame(const realm::Frame::Ptr& frame)
	{
		if (m_gnssPoseFrustum->getNumChildren()==0)
		{
			osg::ref_ptr<osg::Geode> gnss_frustum_geod = 
				createFrustumNode(frame->getCamera(), 60, osg::Vec4(0.,1.,0.,1.));
			m_gnssPoseFrustum->addChild(gnss_frustum_geod);
		}

		if (m_visualPoseFrustum->getNumChildren() == 0)
		{
			osg::ref_ptr<osg::Geode> visual_frustum_geod =
				createFrustumNode(frame->getCamera(), 60, osg::Vec4(1., 0., 0., 1.));
			m_visualPoseFrustum->addChild(visual_frustum_geod);
		}

	}

	void Scene3DNode::subGnssPose(const cv::Mat& pose, uint8_t zone, char band)
	{
		osg::Matrix T = osg::Matrix::translate(
			pose.at<double>(0, 3) - _gnss_base_utm[0],
			pose.at<double>(1, 3) - _gnss_base_utm[1],
			pose.at<double>(2, 3) - _gnss_base_utm[2]);
		osg::Matrix R(
			pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), 0.0,
			pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2), 0.0,
			pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2), 0.0,
			0.0, 0.0, 0.0, 1.0
		);

		osg::Matrix mat = R * T;

		m_gnssPoseFrustum->setMatrix(mat);
	}

	void Scene3DNode::subVisualPose(const cv::Mat& pose, uint8_t zone, char band)
	{
		osg::Matrix T = osg::Matrix::translate(
			pose.at<double>(0, 3) - _gnss_base_utm[0],
			pose.at<double>(1, 3) - _gnss_base_utm[1],
			pose.at<double>(2, 3) - _gnss_base_utm[2]);
		osg::Matrix R(
			pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), 0.0,
			pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2), 0.0,
			pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2), 0.0,
			0.0, 0.0, 0.0, 1.0
		);

		osg::Matrix mat = R * T;

		m_visualPoseFrustum->setMatrix(mat);

		if (m_followFrameViewCLB)
		{
			m_followFrameViewCLB(mat);
		}

	}

	void Scene3DNode::subVisualTraj(const std::vector<cv::Vec3d>& traj)
	{
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;

		for (std::vector<cv::Vec3d>::const_iterator it = traj.cbegin();
			it != traj.cend(); ++it)
		{
			vec_arr->push_back(osg::Vec3(
				(*it)[0] - _gnss_base_utm[0],
				(*it)[1] - _gnss_base_utm[1],
				(*it)[2] - _gnss_base_utm[2]
			));
		}

		m_visualTrajThread->setVertics(*vec_arr.get());
	}

	void Scene3DNode::subGnssTraj(const std::vector<cv::Vec3d>& traj)
	{
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;

		for (std::vector<cv::Vec3d>::const_iterator it = traj.cbegin();
			it != traj.cend(); ++it)
		{
			vec_arr->push_back(osg::Vec3(
				(*it)[0] - _gnss_base_utm[0],
				(*it)[1] - _gnss_base_utm[1],
				(*it)[2] - _gnss_base_utm[2]
			));
		}

		m_gnssTrajThread->setVertics(*vec_arr.get());
	}

	void Scene3DNode::subGnssBase(const cv::Vec3d& gnss_base)
	{
		_gnss_base_wgs84 = gnss_base;

		TJH::SRS::UTM utm = TJH::SRS::UTM::locateUTM(
			TJH::SRS::LonLat(gnss_base[0],
				gnss_base[1]));

		TJH::SRS::Coord3 utm_xyz =
			utm.fromLonLatAlt(TJH::SRS::LonLatAlt(gnss_base[0],
				gnss_base[1], gnss_base[2]));

		_gnss_base_utm = cv::Vec3d(utm_xyz.x(),
			utm_xyz.y(), utm_xyz.z());

	}

	void Scene3DNode::subSparse(const realm::PointCloud::Ptr& sparse_cloud)
	{
		int pt_num = sparse_cloud->size();
		const cv::Mat& data = sparse_cloud->data();
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		vec_arr->resize(pt_num);
		clr_arr->resize(1);
		clr_arr->at(0).set(1.0, 1.0, 0.0, 1.0);
		for (int i = 0; i < pt_num; ++i)
		{
			vec_arr->at(i).set(
				data.at<double>(i, 0) - _gnss_base_utm[0],
				data.at<double>(i, 1) - _gnss_base_utm[1],
				data.at<double>(i, 2) - _gnss_base_utm[2]);
			/*clr_arr->at(i).set(
				data.at<double>(i, 4) / 255.0f,
				data.at<double>(i, 3) / 255.0f,
				data.at<double>(i, 5) / 255.0f,
				1.0f);*/
		}

		m_sparseThread->setVerticsAndColors(*vec_arr.get(), *clr_arr.get());
	}

	void Scene3DNode::subDense(const realm::PointCloud::Ptr& sparse_cloud)
	{
		int pt_num = sparse_cloud->size();
		const cv::Mat& data = sparse_cloud->data();
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		vec_arr->resize(pt_num);
		clr_arr->resize(1);
		clr_arr->at(0).set(1.0, 0.0, 0.0, 1.0);
		for (int i = 0; i < pt_num; ++i)
		{
			vec_arr->at(i).set(
				data.at<double>(i, 0) - _gnss_base_utm[0],
				data.at<double>(i, 1) - _gnss_base_utm[1],
				data.at<double>(i, 2) - _gnss_base_utm[2]);
			/*clr_arr->at(i).set(
				data.at<double>(i, 4) / 255.0f,
				data.at<double>(i, 3) / 255.0f,
				data.at<double>(i, 5) / 255.0f,
				1.0f);*/
		}

		m_denseThread->setVerticsAndColors(*vec_arr.get(), *clr_arr.get());
	}

	//void Scene3DNode::subFaces(const realm::Mesh::Ptr& mesh)
	//{
	//	//osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
	//	//osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;

	//	std::map<std::string, osg::ref_ptr<osg::Vec3Array>> vec_arr_map;
	//	std::map<std::string, osg::ref_ptr<osg::Vec4Array>> clr_arr_map;
	//	std::map<std::string, std::pair<int,int>> tile_row_col_map;

	//	for (int r = -10; r < 10; ++r)
	//	{
	//		for (int c = -10; c < 10; ++c)
	//		{
	//			std::stringstream ss;

	//			ss << "Tile_";
	//			if (c >= 0)
	//			{
	//				ss << "+";
	//			}
	//			else
	//			{
	//				ss << "-";
	//			}
	//			ss << abs(c) << "_";
	//			if (r >= 0)
	//			{
	//				ss << "+";
	//			}
	//			else
	//			{
	//				ss << "-";
	//			}
	//			ss << abs(r);
	//			std::string tile_name = ss.str();
	//			osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
	//			osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
	//			vec_arr_map[tile_name] = vec_arr;
	//			clr_arr_map[tile_name] = clr_arr;
	//			tile_row_col_map[tile_name] = std::pair<int, int>(r, c);
	//		}
	//	}

	//	size_t face_num = mesh->face_size();
	//	size_t vert_num = mesh->vert_size();

	//	for (const auto& face : faces)
	//	{
	//		if (cv::norm(face.vertices[0] - face.vertices[1]) > 5.0)
	//			continue;
	//		if (cv::norm(face.vertices[1] - face.vertices[2]) > 5.0)
	//			continue;

	//		osg::Vec3 pt1;
	//		pt1.x() = face.vertices[0].x - _gnss_base_utm[0];
	//		pt1.y() = face.vertices[0].y - _gnss_base_utm[1];
	//		pt1.z() = face.vertices[0].z - _gnss_base_utm[2];

	//		osg::Vec4 rgba1;
	//		rgba1.b() = static_cast<float>(face.color[0][0]) / 255.0f;
	//		rgba1.g() = static_cast<float>(face.color[0][1]) / 255.0f;
	//		rgba1.r() = static_cast<float>(face.color[0][2]) / 255.0f;
	//		rgba1.a() = 1.0;

	//		osg::Vec3  pt2;
	//		pt2.x() = face.vertices[1].x - _gnss_base_utm[0];
	//		pt2.y() = face.vertices[1].y - _gnss_base_utm[1];
	//		pt2.z() = face.vertices[1].z - _gnss_base_utm[2];

	//		osg::Vec4 rgba2;
	//		rgba2.b() = static_cast<float>(face.color[1][0]) / 255.0f;
	//		rgba2.g() = static_cast<float>(face.color[1][1]) / 255.0f;
	//		rgba2.r() = static_cast<float>(face.color[1][2]) / 255.0f;
	//		rgba2.a() = 1.0;

	//		osg::Vec3 pt3;
	//		pt3.x() = face.vertices[2].x - _gnss_base_utm[0];
	//		pt3.y() = face.vertices[2].y - _gnss_base_utm[1];
	//		pt3.z() = face.vertices[2].z - _gnss_base_utm[2];

	//		osg::Vec4 rgba3;
	//		rgba3.b() = static_cast<float>(face.color[2][0]) / 255.0f;
	//		rgba3.g() = static_cast<float>(face.color[2][1]) / 255.0f;
	//		rgba3.r() = static_cast<float>(face.color[2][2]) / 255.0f;
	//		rgba3.a() = 1.0;

	//		/*vec_arr->push_back(pt1);
	//		vec_arr->push_back(pt2);
	//		vec_arr->push_back(pt3);
	//		clr_arr->push_back(rgba1);
	//		clr_arr->push_back(rgba2);
	//		clr_arr->push_back(rgba3);*/

	//		osg::Vec3 face_center = (pt1 + pt2 + pt3) / 3;

	//		int col = static_cast<int>(face_center.x() / m_tile_size);
	//		int row = static_cast<int>(face_center.y() / m_tile_size);

	//		std::stringstream ss;

	//		ss << "Tile_";
	//		if (col >= 0)
	//		{
	//			ss << "+";
	//		}
	//		else 
	//		{
	//			ss << "-";
	//		}
	//		ss << abs(col) << "_";
	//		if (row >= 0)
	//		{
	//			ss << "+";
	//		}
	//		else
	//		{
	//			ss << "-";
	//		}
	//		ss << abs(row);
	//		std::string tile_name = ss.str();

	//		osg::ref_ptr<osg::Vec3Array> vec_arr = nullptr;
	//		osg::ref_ptr<osg::Vec4Array> clr_arr = nullptr;

	//		std::map<std::string, osg::ref_ptr<osg::Vec3Array>>::iterator vec_arr_it 
	//			= vec_arr_map.find(tile_name);
	//		std::map<std::string, osg::ref_ptr<osg::Vec4Array>>::iterator clr_arr_it
	//			= clr_arr_map.find(tile_name);

	//		if (vec_arr_it != vec_arr_map.end())
	//		{
	//			vec_arr = vec_arr_it->second;
	//			clr_arr = clr_arr_it->second;
	//		}
	//		else
	//		{
	//			vec_arr = new osg::Vec3Array;
	//			clr_arr = new osg::Vec4Array;
	//			vec_arr_map[tile_name] = vec_arr;
	//			clr_arr_map[tile_name] = clr_arr;
	//			tile_row_col_map[tile_name] = std::pair<int, int>(row, col);
	//		}

	//		vec_arr->push_back(pt1);
	//		vec_arr->push_back(pt2);
	//		vec_arr->push_back(pt3);
	//		clr_arr->push_back(rgba1);
	//		clr_arr->push_back(rgba2);
	//		clr_arr->push_back(rgba3);
	//	}

	//	for (std::map<std::string, osg::ref_ptr<osg::Vec3Array>>::iterator
	//		vec_arr_it = vec_arr_map.begin(); vec_arr_it != vec_arr_map.end(); ++vec_arr_it)
	//	{
	//		std::string tile_name = vec_arr_it->first;
	//		std::map<std::string, osg::ref_ptr<osg::Vec4Array>>::iterator clr_arr_it
	//			= clr_arr_map.find(tile_name);
	//		std::map<std::string, std::pair<int, int>>::iterator row_col_it = tile_row_col_map.find(tile_name);

	//		osg::ref_ptr<osg::Vec3Array> vec_arr = vec_arr_it->second;
	//		osg::ref_ptr<osg::Vec4Array> clr_arr = clr_arr_it->second;
	//		std::pair<int, int> row_col_pair = row_col_it->second;

	//		if (vec_arr->size() == 0) { continue; }

	//		osg::ref_ptr<TileFacesNode> tileFaceNode = nullptr;
	//		TileFacesNodeMap::iterator tile_it = m_tileFacesNodes.find(tile_name);
	//		if (tile_it != m_tileFacesNodes.end())
	//		{
	//			tileFaceNode = tile_it->second;
	//		}
	//		else
	//		{
	//			//tileFaceNode = TileFacesNode::createTileFacesNode(
	//			//	row_col_pair.first, row_col_pair.second, m_tile_size);

	//			//// m_faces->addChild(tileFaceNode->facesGeode());

	//			//m_tileFacesNodes[tile_name] = tileFaceNode;
	//			//tileFaceNode->facesThread()->startThread();

	//			continue;
	//		}

	//		tileFaceNode->facesThread()->setVerticsAndColors(*vec_arr, *clr_arr);
	//	}


	//}


	void Scene3DNode::subMesh(const realm::Mesh::Ptr& mesh)
	{
		return;
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		osg::ref_ptr<osg::Vec2Array> tex_arr = new osg::Vec2Array;
		osg::ref_ptr<osg::UIntArray> ind_arr = new osg::UIntArray;
	
		size_t face_num = mesh->face_size();
		size_t vert_num = mesh->vert_size();

		cv::Point3d* verts = mesh->vertices();
		cv::Vec4b* clrs = mesh->colors();
		cv::Vec2f* texCoords = mesh->texCoords();
		size_t* faces = mesh->faces();
		cv::Mat& cvTex = mesh->texture();

		vec_arr->resize(vert_num);
		clr_arr->resize(vert_num);
		tex_arr->resize(vert_num);
		ind_arr->resize(face_num * 3);

		for (size_t i = 0; i < vert_num; i++)
		{
			(*vec_arr)[i].set(
				verts[i].x - _gnss_base_utm[0],
				verts[i].y - _gnss_base_utm[1],
				verts[i].z - _gnss_base_utm[2]);

			(*clr_arr)[i].set(
				static_cast<float>(clrs[i][0]) / 255.0f,
				static_cast<float>(clrs[i][1]) / 255.0f,
				static_cast<float>(clrs[i][2]) / 255.0f,
				1.0);

			(*tex_arr)[i].set(texCoords[i][0], texCoords[i][1]);
		}

		for (size_t i = 0; i < face_num; i++)
		{
			(*ind_arr)[i * 3 + 0] = static_cast<GLuint>(faces[i * 3 + 0]);
			(*ind_arr)[i * 3 + 1] = static_cast<GLuint>(faces[i * 3 + 1]);
			(*ind_arr)[i * 3 + 2] = static_cast<GLuint>(faces[i * 3 + 2]);
		}

		osg::ref_ptr<osg::Image> osg_texImg = new osg::Image();
		osg_texImg->allocateImage(cvTex.cols, cvTex.rows, 4, GL_BGRA, GL_UNSIGNED_BYTE, 1);
		unsigned char* im_data = osg_texImg->data();
		memcpy(im_data, cvTex.data, cvTex.cols * cvTex.rows * 4 * sizeof(unsigned char));
		osg_texImg->flipVertical();


		osg::ref_ptr<osg::Geode> newMeshGeode = createMeshGeode(vec_arr, clr_arr,
			tex_arr, ind_arr, osg_texImg);

		osg::Node* old_MeshNode = m_mesh->getChild(0);
		m_mesh->replaceChild(old_MeshNode, newMeshGeode);

		std::string mesh_node_filename = _mesh_output_directory + "/mesh.osgb";

		osgDB::writeNodeFile(*m_mesh.get(), mesh_node_filename);
	}


	void Scene3DNode::subOrtho(const realm::CvGridMap& map, uint8_t zone, char band)
	{

	}

	void Scene3DNode::subElevation(const realm::CvGridMap& map, uint8_t zone, char band)
	{

	}

}
