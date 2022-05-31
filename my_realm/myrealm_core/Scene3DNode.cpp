#include "Scene3DNode.h"
#include "MyREALM.h"
#include <glog/logging.h>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
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

		_gnss_base_wgs84 = cv::Vec3d(0, 0, 0);
		_gnss_base_utm = cv::Vec3d(0, 0, 0);
		m_visualTrajThread = new TrajReceiverThread;
		m_gnssTrajThread = new TrajReceiverThread;
		m_sparseThread = new SparseReceiverThread;
		m_facesThread = new FacesReceiverThread;

		m_visualTrajCLB = new TrajDrawCallback(m_visualTrajThread);
		m_gnssTrajCLB = new TrajDrawCallback(m_gnssTrajThread);
		m_sparseCLB = new SparseDrawCallback(m_sparseThread);
		m_facesCLB = new  FacesDrawCallback(m_facesThread);

		m_visualTraj = createTrajGeode(osg::Vec4(1, 0, 0, 1), 2,
			m_visualTrajCLB);
		m_gnssTraj = createTrajGeode(osg::Vec4(0, 1, 0, 1), 2,
			m_gnssTrajCLB);
		m_sparse = createSparseGeode(osg::Vec4(1, 1, 0, 1), 2,
			m_sparseCLB);
		m_faces = createFacesGeode(m_facesCLB);

		m_visualTrajThread->startThread();
		m_gnssTrajThread->startThread();
		m_sparseThread->startThread();
		m_facesThread->startThread();

		_topic_prefix = "/realm/" + _id_camera ;

		_topic_visual_traj_in= "/realm/"+ _id_camera +"/pose_estimation/pose/visual/traj";

		_topic_gnss_traj_in = "/realm/" + _id_camera + "/pose_estimation/pose/gnss/traj";

		_topic_gnss_base_in = "/realm/" + _id_camera + "/general/gnss_base";


		_topic_sparse_in = "/realm/" + _id_camera + "/densification/pointcloud";

		_topic_faces_in = "/realm/" + _id_camera + "/mosaicing/mesh";

		
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

		_sub_faces = MyRealmSys::get_instance().getOrCreatePublisher(_topic_faces_in)
			->registSubscriber(_topic_faces_in + "/sub");

		SubFacesFun subFacesFunc = std::bind(&Scene3DNode::subFaces, this, std::placeholders::_1);
		_sub_faces->bindSubFacesFunc(subFacesFunc);

	}

	Scene3DNode::~Scene3DNode()
	{
		/*if(m_visualTraj->getNumChildren()==1) 
			m_visualTraj->getChild(0)->removeUpdateCallback(m_visualTrajCLB);
		if (m_gnssTraj->getNumChildren() == 1) 
			m_gnssTraj->getChild(0)->removeUpdateCallback(m_gnssTrajCLB);
		if (m_sparse->getNumChildren() == 1) 
			m_sparse->getChild(0)->removeUpdateCallback(m_sparseCLB);
		if (m_faces->getNumChildren() == 1) 
			m_faces->getChild(0)->removeUpdateCallback(m_facesCLB);*/

		m_visualTraj->removeChildren(0, m_visualTraj->getNumChildren());
		m_gnssTraj->removeChildren(0, m_gnssTraj->getNumChildren());
		m_sparse->removeChildren(0, m_sparse->getNumChildren());
		m_faces->removeChildren(0, m_faces->getNumChildren());

		if (isRunning())
		{
			this->cancel();
		}

		m_visualTrajThread->cancel();
		m_gnssTrajThread->cancel();
		m_sparseThread->cancel();
		m_facesThread->cancel();

		delete m_visualTrajThread;
		delete m_gnssTrajThread;
		delete m_sparseThread;
		delete m_facesThread;

		m_visualTrajThread = nullptr;
		m_gnssTrajThread = nullptr;
		m_sparseThread = nullptr;
		m_facesThread = nullptr;

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
		if (m_facesThread->isRunning())
		{
			m_facesThread->cancel();
		}

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

	osg::Geode* Scene3DNode::visualTrajNode()
	{
		return m_visualTraj;
	}

	osg::Geode* Scene3DNode::gnssTrajNode()
	{
		return m_gnssTraj;
	}

	osg::Geode* Scene3DNode::sparseNode()
	{
		return m_sparse.get();
	}

	osg::Geode* Scene3DNode::facesNode()
	{
		return m_faces.get();
	}

	void Scene3DNode::readParams()
	{
		_node_paras.param("config/id", _id_node, std::string("uninitialised"));
		_node_paras.param("config/opt/working_directory", _path_working_directory, std::string("uninitialised"));

		_id_camera = _id_node;

		if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
			throw(std::invalid_argument("Error: Working directory does not exist!"));
	}

	void Scene3DNode::readStageSettings()
	{
	}

	void Scene3DNode::setPaths()
	{

	}

	void Scene3DNode::subFrame(const realm::Frame::Ptr& frame)
	{

	}

	void Scene3DNode::subPose(const cv::Mat& pose)
	{
		osg::Matrix osg_mat;

		osg::Matrix T = osg::Matrix::translate(
			pose.at<double>(0, 3),
			pose.at<double>(1, 3),
			pose.at<double>(2, 3));
		osg::Matrix R(
			pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), 0.0,
			pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2), 0.0,
			pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2), 0.0,
			0.0, 0.0, 0.0, 1.0
		);

		osg::Matrix mat = R * T;

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
		return;
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
	    const cv::Mat& data = sparse_cloud->data();
		int pt_num = sparse_cloud->size();
		for (int i = 0; i < pt_num; ++i)
		{
			vec_arr->push_back(osg::Vec3(
				data.at<double>(i,0) - _gnss_base_utm[0],
				data.at<double>(i,1) - _gnss_base_utm[1],
				data.at<double>(i,2) - _gnss_base_utm[2]
			
			));
		}

		m_sparseThread->setVertics(*vec_arr);

	}

	void Scene3DNode::subFaces(const std::vector<realm::Face>& faces)
	{
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;

		for (const auto& face : faces)
		{
			if (cv::norm(face.vertices[0] - face.vertices[1]) > 5.0)
				continue;
			if (cv::norm(face.vertices[1] - face.vertices[2]) > 5.0)
				continue;

			osg::Vec3 pt1;
			pt1.x() = face.vertices[0].x - _gnss_base_utm[0];
			pt1.y() = face.vertices[0].y - _gnss_base_utm[1];
			pt1.z() = face.vertices[0].z - _gnss_base_utm[2];

			osg::Vec4 rgba1;
			rgba1.b() = static_cast<float>(face.color[0][0]) / 255.0f;
			rgba1.g() = static_cast<float>(face.color[0][1]) / 255.0f;
			rgba1.r() = static_cast<float>(face.color[0][2]) / 255.0f;
			rgba1.a() = 1.0;

			osg::Vec3  pt2;
			pt2.x() = face.vertices[1].x - _gnss_base_utm[0];
			pt2.y() = face.vertices[1].y - _gnss_base_utm[1];
			pt2.z() = face.vertices[1].z - _gnss_base_utm[2];

			osg::Vec4 rgba2;
			rgba2.b() = static_cast<float>(face.color[1][0]) / 255.0f;
			rgba2.g() = static_cast<float>(face.color[1][1]) / 255.0f;
			rgba2.r() = static_cast<float>(face.color[1][2]) / 255.0f;
			rgba2.a() = 1.0;

			osg::Vec3 pt3;
			pt3.x() = face.vertices[2].x - _gnss_base_utm[0];
			pt3.y() = face.vertices[2].y - _gnss_base_utm[1];
			pt3.z() = face.vertices[2].z - _gnss_base_utm[2];

			osg::Vec4 rgba3;
			rgba3.b() = static_cast<float>(face.color[2][0]) / 255.0f;
			rgba3.g() = static_cast<float>(face.color[2][1]) / 255.0f;
			rgba3.r() = static_cast<float>(face.color[2][2]) / 255.0f;
			rgba3.a() = 1.0;

			vec_arr->push_back(pt1);
			vec_arr->push_back(pt2);
			vec_arr->push_back(pt3);
			clr_arr->push_back(rgba1);
			clr_arr->push_back(rgba2);
			clr_arr->push_back(rgba3);
		}

		m_facesThread->setVerticsAndColors(*vec_arr, *clr_arr);
	}

}
