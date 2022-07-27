#include "Scene3DNode.h"
#include "MyREALM.h"
#include <sstream>
#include <fstream>
#include <iomanip>
#include <glog/logging.h>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
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
		m_update_mesh_count = 0;
		m_followFrameViewCLB = NULL;

		_gnss_base_inited = false;
		_gnss_base_wgs84 = cv::Vec3d(0, 0, 0);
		_gnss_base_utm = cv::Vec3d(0, 0, 0);
		m_visualTrajThread = new TrajReceiverThread;
		m_gnssTrajThread = new TrajReceiverThread;
		m_sparseThread = new SparseReceiverThread;
		m_denseThread = new SparseReceiverThread;
		m_meshThread = new MeshReceiverThread;
		m_arThread = new ArImageReceiverThread;

		m_visualTrajCLB = new TrajDrawCallback(m_visualTrajThread);
		m_gnssTrajCLB = new TrajDrawCallback(m_gnssTrajThread);
		m_sparseCLB = new SparseDrawCallback(m_sparseThread);
		m_denseCLB = new SparseDrawCallback(m_denseThread);
		m_meshCLB = new  MeshNodeCallback(m_meshThread);
		m_arCLB = new ArImageUpdateCallback(m_arThread);

		m_visualTraj = createTrajGeode(osg::Vec4(1, 0, 0, 1), 2,
			m_visualTrajCLB);
		m_gnssTraj = createTrajGeode(osg::Vec4(0, 1, 0, 1), 2,
			m_gnssTrajCLB);
		m_sparse = createSparseGeode(osg::Vec4(1, 1, 0, 1), 2,
			m_sparseCLB);
		m_dense = createSparseGeode(osg::Vec4(1, 0, 0, 1), 4,
			m_denseCLB);
		m_mesh = new osg::Group;
		//m_mesh->setUpdateCallback(m_meshCLB);

		m_gnssPoseFrustum = new osg::MatrixTransform;
		m_visualPoseFrustum = new osg::MatrixTransform;
		m_arNode = new osg::MatrixTransform;
		m_arImg = nullptr;

		osg::ref_ptr<osg::Vec3Array> mesh_vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> mesh_clr_arr = new osg::Vec4Array;
		osg::ref_ptr<osg::Vec2Array> mesh_texture_arr = new osg::Vec2Array;
		osg::ref_ptr<osg::UIntArray> mesh_face_ary = new osg::UIntArray;

		m_visualTrajThread->startThread();
		m_gnssTrajThread->startThread();
		m_sparseThread->startThread();
		m_denseThread->startThread();
		m_meshThread->startThread();
		//m_arThread->startThread();

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
		_topic_update_elevation_ortho_in = "/realm/" + _id_camera + "/mosaicing/update/elevation_ortho";

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
		


		_sub_elevation_ortho = MyRealmSys::get_instance().getOrCreatePublisher(_topic_update_elevation_ortho_in)
			->registSubscriber(_topic_update_elevation_ortho_in + "/sub");

		SubCvGridMapFun subElevOrthoFunc = std::bind(&Scene3DNode::subUpdateElevationOrtho, this, 
			std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		_sub_elevation_ortho->bindCvGridMapFunc(subElevOrthoFunc);

		

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

		if (m_arNode->getNumChildren() == 1)
		{
			osg::Geode* arGeode = m_arNode->getChild(0)->asGeode();
			removeArImageUpdateCallback(arGeode, m_arCLB);
		}

		//m_mesh->removeUpdateCallback(m_meshCLB);
		
		/*if (m_faces->getNumChildren() == 1)
			m_faces->getChild(0)->removeUpdateCallback(m_facesCLB);*/
		
		/*m_visualTraj->removeChildren(0, m_visualTraj->getNumChildren());
		m_gnssTraj->removeChildren(0, m_gnssTraj->getNumChildren());
		m_sparse->removeChildren(0, m_sparse->getNumChildren());
		m_mesh->removeChildren(0, m_mesh->getNumChildren());*/
		//m_faces->removeChildren(0, m_faces->getNumChildren());

		m_arNode->removeChildren(0, m_mesh->getNumChildren());
		m_arNode = nullptr;

		if (isRunning())
		{
			this->cancel(); 
		}

		
		m_visualTrajThread->cancel();
		m_gnssTrajThread->cancel();
		m_sparseThread->cancel();
		m_denseThread->cancel();
		m_meshThread->cancel();
		m_arThread->cancel();

		delete m_visualTrajThread;
		delete m_gnssTrajThread;
		delete m_sparseThread;
		delete m_denseThread;
		delete m_meshThread;
		delete m_arThread;

		m_visualTrajThread = nullptr;
		m_gnssTrajThread = nullptr;
		m_sparseThread = nullptr;
		m_denseThread = nullptr;
		m_meshThread = nullptr;
		m_arThread = nullptr;

	}


	int Scene3DNode::cancel()
	{
		_done = true;

		//if (m_mesh->getNumChildren() > 0)
		//{
		//	// 保存mesh
		//	std::string save_osgb_filepath = _mesh_output_directory + "/mesh.osgb";
		//	osgDB::writeNodeFile(*(m_mesh.get()), save_osgb_filepath);
		//}

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
		if (m_arThread->isRunning())
		{
			m_arThread->cancel();
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
		m_arNode->removeChildren(0, m_arNode->getNumChildren());
		m_arImg = nullptr;
		m_followFrameViewCLB = clb;
	}

	void Scene3DNode::writeMeshSrs(const std::string& filename)
	{
		std::ofstream fs(filename.c_str());
		if (fs.fail())
		{
			return;
		}

		fs << "SRS:        " << _gnss_base_utm_name << " (EPSG:" << _gnss_base_utm_epsg << ")\n";
		fs << "SRS_Origin: " << std::fixed << std::setprecision(6) << _gnss_base_utm[0] << ",  " << _gnss_base_utm[1] << ",  " << _gnss_base_utm[2] << "\n";
		fs.flush();
		fs.close();
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

	osg::Node* Scene3DNode::arNode()
	{
		return m_arNode->asNode();
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

		if (m_followFrameViewCLB)
		{
			cv::Mat img_init = frame->getImageRaw();
			cv::Mat img0, img;
			cv::cvtColor(img_init, img0, cv::COLOR_BGR2RGB);

			cv::resize(img0, img0, cv::Size(img0.cols * 0.4, img0.rows * 0.4));
			
			if (m_arNode->getNumChildren() == 0)
			{
				osg::ref_ptr<osg::Geode> ar_geod = generateARImageNode(frame->getCamera(), 
					60, m_arCLB);
				m_arNode->addChild(ar_geod);

				m_arThread->startThread();
			}

			m_arThread->setImage(img0.cols, img0.rows, GL_RGB, GL_UNSIGNED_BYTE, img0.data);

			cv::Mat pose = frame->getDefaultPose();
			

			osg::Matrix T = osg::Matrix::translate(
				pose.at<double>(0, 3) - _gnss_base_utm[0],
				pose.at<double>(1, 3) - _gnss_base_utm[1],
				pose.at<double>(2, 3) - _gnss_base_utm[2]);
			osg::Matrix R(
				pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), 0.0,
				pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2), 0.0,
				pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2), 0.0,
				0.0, 0.0, 0.0, 1.0);

			osg::Matrix mat = R * T;

			m_arNode->setMatrix(mat);

			osg::Vec3 eye = osg::Vec3(0.0, 0.0, 0.0) * mat;
			osg::Vec3 center = osg::Vec3(0.0, 0.0, 1.0) * mat;
			osg::Vec3 vect = center - eye;
			osg::Vec3 up = osg::Vec3(0.0, -1.0, 0.0) * R;
			up.normalize();

			osg::Matrix mat2;
			mat2.makeLookAt(eye, center, up);

			double cam_f = frame->getCamera()->fx();
			double cam_w = frame->getCamera()->width();
			double cam_h = frame->getCamera()->height();
			double cam_cx = frame->getCamera()->cx();
			double cam_cy = frame->getCamera()->cy();

			m_followFrameViewCLB(mat2, cam_w, cam_h, cam_f, cam_cx, cam_cy);
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

		_gnss_base_utm_name = utm.name();
		_gnss_base_utm_epsg = utm.epsg();
		_gnss_base_inited = true;

		if (_gnss_base_inited)
		{
			std::string save_srs_filepath = _mesh_output_directory + "/SRS.txt";
			writeMeshSrs(save_srs_filepath);
		}
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
		cv::Mat cvTexRGBA;
		cv::cvtColor(cvTex, cvTexRGBA, cv::COLOR_BGRA2RGBA);

		osg::ref_ptr<osg::Image> osg_texImg = new osg::Image();
		osg_texImg->allocateImage(cvTexRGBA.cols, cvTexRGBA.rows, 1, GL_RGBA, GL_UNSIGNED_BYTE, 1);

		unsigned int newTotalSize = osg_texImg->computeRowWidthInBytes(cvTexRGBA.cols,
			GL_RGBA, GL_UNSIGNED_BYTE, 1) * cvTexRGBA.rows * 1;
		unsigned char* im_data = osg_texImg->data();
		memcpy(im_data, cvTexRGBA.data, newTotalSize * sizeof(unsigned char));
		osg_texImg->flipVertical();

		std::string save_osgb_filepath = "";
		if (m_update_mesh_count % 2 == 0)
		{
			save_osgb_filepath = _mesh_output_directory + "/mesh_update.osgb";
		}

		m_meshThread->updateMeshGeode(vec_arr, clr_arr,
			tex_arr, ind_arr, osg_texImg, save_osgb_filepath);

		
		m_update_mesh_count++;
	}

	void Scene3DNode::subUpdateElevationOrtho(
		const realm::CvGridMap& map, uint8_t zone, char band)
	{

		if (!m_tile_mesh)
		{
			std::string tile_lod_dir = _mesh_output_directory + "/mesh";
			if (!io::dirExists(tile_lod_dir))
				io::createDir(tile_lod_dir);

			m_tile_mesh = std::make_shared<TileMesh>(
				osg::Vec2(0, 0), 500.0f, 0.8f, 4,
				tile_lod_dir, 4);

			osg::ref_ptr<osg::Group> tileMeshGroup = m_tile_mesh->meshNode();
			osg::StateSet* tile_SS = tileMeshGroup->getOrCreateStateSet();
			tile_SS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

			m_mesh->addChild(tileMeshGroup);
		}

		osg::Vec2 roi = osg::Vec2(map.roi().x, map.roi().y)
			- osg::Vec2(_gnss_base_utm[0], _gnss_base_utm[1]);
		
		if (map.exists("elevation") && map.exists("color_rgb")) 
		{
			TileGridMap::Ptr dom = std::make_shared<TileGridMap>(map.size().width,
				map.size().height, CV_8UC3, roi, map.resolution());
			cv::cvtColor(map["color_rgb"], dom->map, cv::COLOR_RGBA2RGB);

			TileGridMap::Ptr dsm = std::make_shared<TileGridMap>(map.size().width,
				map.size().height, CV_32FC1, roi, map.resolution());
			map["elevation"].copyTo(dsm->map);

			m_tile_mesh->updateTileGridMap(dsm, dom);
		}
	}


}
