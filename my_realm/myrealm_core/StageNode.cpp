#include "StageNode.h"
#include "MyREALM.h"
#include "Conversions.h"
#include <OpenThreads/ScopedLock>
#include <glog/logging.h>
#include <thread>

using namespace realm;

namespace MyREALM
{
	StageNode::StageNode(const NodeParas& node_paras)
		:OpenThreads::Thread(),
		_node_paras(node_paras),
		_sub_input_frame(NULL),
		_sub_input_imu(NULL),
		_sub_output_dir(NULL),
		_is_master_stage(false),
		_do_shutdown(false),
		_is_stage_running(false),
		_gnss_base_initialized(false)
	{
		// Read basic launch file inputs
		readParams();

		// Specify stage
		setPaths();
		readStageSettings();

		// Set naming conventions
		_topic_prefix = "/realm/" + _id_camera + "/" + _type_stage + "/";

		_sub_input_frame = MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_in)
			->registSubscriber("/realm/" + _id_camera + "/" + _type_stage + "/sub/frame");

		SubFrameFun subFrameFunc = std::bind(&StageNode::subFrame, this, std::placeholders::_1);
		_sub_input_frame->bindSubFrameFunc(subFrameFunc);

		if (_is_master_stage)
		{
			_publisher.insert({ "general/output_dir", MyRealmSys::get_instance()
				.getOrCreatePublisher("/realm/" + _id_camera + "/general/output_dir") });
			_publisher.insert({ "general/gnss_base", MyRealmSys::get_instance()
				.getOrCreatePublisher("/realm/" + _id_camera + "/general/gnss_base") });
		}
		else
		{
			_sub_output_dir = MyRealmSys::get_instance()
				.getOrCreatePublisher("/realm/" + _id_camera + "/general/output_dir")
				->registSubscriber("/realm/" + _id_camera + "/" + _type_stage + "/sub/output_dir");

			SubOutDirFun subOutDirFunc = std::bind(&StageNode::subOutputPath, this, std::placeholders::_1);
			_sub_output_dir->bindSubOutDirFunc(subOutDirFunc);
		}

		// Provide camera information a priori to all stages
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Loading camera from path:\n\t"
			<< _file_settings_camera.c_str();
		_settings_camera = CameraSettingsFactory::load(_file_settings_camera);
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Detected camera model: \'"
			<< (*_settings_camera)["type"].toString().c_str() << "\'";

		// Create stages
		if (_type_stage == "pose_estimation")
			createStagePoseEstimation();
		if (_type_stage == "densification")
			createStageDensification();
		if (_type_stage == "surface_generation")
			createStageSurfaceGeneration();
		if (_type_stage == "ortho_rectification")
			createStageOrthoRectification();
		if (_type_stage == "mosaicing")
			createStageMosaicing();
		if (_type_stage == "tileing")
			createStageTileing();

		// set stage path if master stage
		if (_is_master_stage)
			_stage->initStagePath(_path_output + "/" + _dir_date_time);

	}

	StageNode::~StageNode()
	{
		if (_is_stage_running)
		{
			this->srvFinish();
		}

		while (isRunning()) YieldCurrentThread();

	}

	void StageNode::spin()
	{
		if (_is_master_stage)
		{
			// Share output folder with slaves
			_publisher["general/output_dir"]->pubOutputDir(_dir_date_time);
		}

		if (_gnss_base_initialized && _is_master_stage)
		{
			// Master stage sends first tf as mission reference
			_publisher["general/gnss_base"]->pubGnssBase(_gnss_base);
		}
	}


	int StageNode::cancel()
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Canceling stage...!";
		_stage->requestFinish();
		_stage->join();
		_is_stage_running = false;
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully cancel stage!";
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Shutting stage node down...";
		std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
		_do_shutdown = true;
		return true;
	}

	void StageNode::run()
	{
		_gnss_base_initialized = false;
		_do_shutdown = false;
		_stage->start();
		_is_stage_running = true;
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Started stage node successfully!";

		do
		{
			this->spin();
			OpenThreads::Thread::microSleep(50000);
		} while (!_do_shutdown);

		if (_is_stage_running)
		{
			_stage->requestFinish();
			_stage->join();
			_is_stage_running = false;
		}

	}

	void StageNode::pause()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		this->srvStop();
	}

	void StageNode::resume()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		this->srvResume();
	}

	std::string StageNode::outputDirDatetime() const
	{
		return _path_output + "/" + _dir_date_time;
	}

	void StageNode::createStagePoseEstimation()
	{
		// Pose estimation uses external frameworks, therefore load settings for that
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Loading vslam settings from path:\n\t" << _file_settings_method.c_str();
		VisualSlamSettings::Ptr settings_vslam = VisualSlamSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Detected vslam type: '" << (*settings_vslam)["type"].toString().c_str() << "'";

		ImuSettings::Ptr settings_imu = nullptr;
		if ((*_settings_stage)["use_imu"].toInt() > 0)
		{
			settings_imu = std::make_shared<ImuSettings>();
			settings_imu->loadFromFile(_file_settings_imu);
		}

		// Topic and stage creation
		_stage = std::make_shared<stages::PoseEstimation>(_settings_stage, settings_vslam, 
			_settings_camera, settings_imu, (*_settings_camera)["fps"].toDouble());

		_publisher.insert({ "output/frame", MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_out) });
		_publisher.insert({ "output/pose/visual/utm", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/visual/utm") });
		_publisher.insert({ "output/pose/visual/wgs", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/visual/wgs") });
		_publisher.insert({ "output/pose/visual/traj", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/visual/traj") });
		_publisher.insert({ "output/pose/gnss/utm", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/gnss/utm") });
		_publisher.insert({ "output/pose/gnss/wgs", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/gnss/wgs") });
		_publisher.insert({ "output/pose/gnss/traj", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/gnss/traj") });
		_publisher.insert({ "output/pointcloud", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pointcloud") });
		_publisher.insert({ "debug/tracked", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "tracked") });
		linkStageTransport();

		if (_topic_imu_in != "uninitialised")
		{
			//_sub_input_imu = _nh.subscribe(_topic_imu_in, 100, &StageNode::subImu, this, ros::TransportHints());

			_sub_input_imu = MyRealmSys::get_instance().getOrCreatePublisher(_topic_imu_in)
				->registSubscriber(_topic_prefix + "sub/imu");

			SubImuFun subImuFunc = std::bind(&StageNode::subImu, this, std::placeholders::_1);
			_sub_input_imu->bindSubImuFunc(subImuFunc);

		}
	}

	void StageNode::createStageDensification()
	{
		// Densification uses external frameworks, therefore load settings for that
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Loading densifier settings from path:\n\t" << _file_settings_method.c_str();
		DensifierSettings::Ptr settings_densifier = DensifierSettingsFactory::load(_file_settings_method, _path_profile + "/" + _type_stage + "/method");
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: : Detected densifier type: \'" << (*settings_densifier)["type"].toString().c_str() << "\'";

		// Topic and stage creation
		_stage = std::make_shared<stages::Densification>(_settings_stage, settings_densifier, (*_settings_camera)["fps"].toDouble());
		_publisher.insert({ "output/frame", MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_out) });
		_publisher.insert({ "output/pose/utm", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/utm") });
		_publisher.insert({ "output/pose/wgs", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/wgs") });
		_publisher.insert({ "output/pose/traj", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pose/traj") });
		_publisher.insert({ "output/pointcloud", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pointcloud") });
		_publisher.insert({ "output/img_rectified", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "img") });
		_publisher.insert({ "output/depth", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "depth") });
		_publisher.insert({ "output/depth_display", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "depth_display") });
		linkStageTransport();
	}

	void StageNode::createStageSurfaceGeneration()
	{
		_stage = std::make_shared<stages::SurfaceGeneration>(_settings_stage, (*_settings_camera)["fps"].toDouble());
		_publisher.insert({ "output/frame", MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_out) });
		_publisher.insert({ "output/elevation_map", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "elevation_map") });
		linkStageTransport();
	}

	void StageNode::createStageOrthoRectification()
	{
		_stage = std::make_shared<stages::OrthoRectification>(_settings_stage, (*_settings_camera)["fps"].toDouble());
		_publisher.insert({ "output/frame", MyRealmSys::get_instance().getOrCreatePublisher(_topic_frame_out) });
		_publisher.insert({ "output/rectified", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "rectified") });
		_publisher.insert({ "output/pointcloud", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pointcloud") });
		linkStageTransport();
	}

	void StageNode::createStageMosaicing()
	{
		_stage = std::make_shared<stages::Mosaicing>(_settings_stage, (*_settings_camera)["fps"].toDouble());
		_publisher.insert({ "output/rgb", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "rgb") });
		_publisher.insert({ "output/elevation",MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "elevation") });
		_publisher.insert({ "output/pointcloud", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pointcloud") });
		_publisher.insert({ "output/mesh", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "mesh") });
		_publisher.insert({ "output/update/ortho", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "update/ortho") });
		_publisher.insert({ "output/full/ortho", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "full/ortho") });
		// _publisher.insert({ "output/update/elevation", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "update/elevation", 5)});
		linkStageTransport();
	}

	void StageNode::createStageTileing()
	{
		_stage = std::make_shared<stages::Tileing>(_settings_stage, (*_settings_camera)["fps"].toDouble());
		//_publisher.insert({"output/rgb", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "rgb")});
		//_publisher.insert({"output/elevation", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "elevation")});
		//_publisher.insert({"output/pointcloud", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "pointcloud")});
		//_publisher.insert({"output/mesh", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "mesh")});
		//_publisher.insert({"output/update/ortho", MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "update/ortho")});
		linkStageTransport();
	}

	void StageNode::linkStageTransport()
	{
		namespace ph = std::placeholders;
		auto transport_frame = std::bind(&StageNode::pubFrame, this, ph::_1, ph::_2);
		auto transport_pose = std::bind(&StageNode::pubPose, this, ph::_1, ph::_2, ph::_3, ph::_4);
		auto transport_pointcloud = std::bind(&StageNode::pubPointCloud, this, ph::_1, ph::_2);
		auto transport_img = std::bind(&StageNode::pubImage, this, ph::_1, ph::_2);
		auto transport_depth = std::bind(&StageNode::pubDepthMap, this, ph::_1, ph::_2);
		auto transport_mesh = std::bind(&StageNode::pubMesh, this, ph::_1, ph::_2);
		auto transport_cvgridmap = std::bind(&StageNode::pubCvGridMap, this, ph::_1, ph::_2, ph::_3, ph::_4);
		_stage->registerFrameTransport(transport_frame);
		_stage->registerPoseTransport(transport_pose);
		_stage->registerPointCloudTransport(transport_pointcloud);
		_stage->registerImageTransport(transport_img);
		_stage->registerDepthMapTransport(transport_depth);
		_stage->registerMeshTransport(transport_mesh);
		_stage->registerCvGridMapTransport(transport_cvgridmap);
	}

	void StageNode::subFrame(const realm::Frame::Ptr& frame)
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Received frame.";
	
		realm::camera::Pinhole::Ptr frame_cam = 
			std::make_shared<realm::camera::Pinhole>(*frame->getCamera());

		Frame::Ptr frame_stage = std::make_shared<Frame>
			(frame->getCameraId(), frame->getFrameId(),
				frame->getTimestamp(), frame->getImageRaw(),
				frame->getGnssUtm(), frame_cam,
				frame->getOrientation());

		frame_stage->setKeyframe(frame->isKeyframe());

		if (frame->hasAccuratePose())
		{
			frame_stage->setVisualPose(frame->getVisualPose());
		}

		if (frame->isGeoreferenced())
		{
			frame_stage->updateGeoreference(frame->getGeoreference());
		}

		if (frame->getSurfaceModel())
		{
			realm::CvGridMap::Ptr map = std::make_shared<realm::CvGridMap>();
			map->setGeometry(frame->getSurfaceModel()->roi(), frame->getSurfaceModel()->resolution());
			std::vector<cv::String> lyr_names = frame->getSurfaceModel()->getAllLayerNames();

			for (const auto& lyr_name : lyr_names)
			{
				map->add(lyr_name, (*frame->getSurfaceModel())[lyr_name]);
			}
			frame_stage->setSurfaceModel(map);
		}
		if (frame->getDepthmap())
		{
			realm::camera::Pinhole::Ptr depth_cam = std::make_shared<realm::camera::Pinhole>(
				*frame->getDepthmap()->getCamera());
			realm::Depthmap::Ptr depth_map = std::make_shared<realm::Depthmap>(
				frame->getDepthmap()->data(), *depth_cam);

			frame_stage->setDepthmap(depth_map);
		}
		if (frame->getOrthophoto())
		{
			realm::CvGridMap::Ptr map = std::make_shared<realm::CvGridMap>();
			map->setGeometry(frame->getOrthophoto()->roi(), frame->getOrthophoto()->resolution());
			std::vector<cv::String> lyr_names = frame->getOrthophoto()->getAllLayerNames();

			for (const auto& lyr_name : lyr_names)
			{
				map->add(lyr_name, (*frame->getOrthophoto())[lyr_name]);
			}
			frame_stage->setOrthophoto(map);
		}
		if (frame->getSurfaceAssumption() == realm::SurfaceAssumption::ELEVATION)
		{
			frame_stage->setSurfaceAssumption(realm::SurfaceAssumption::ELEVATION);
		}

		if (frame->getSparseCloud())
		{
			if (!frame->getSparseCloud()->empty())
			{
				std::vector<uint32_t> point_ids =
					frame->getSparseCloud()->getPointIds();
				const cv::Mat& points = frame->getSparseCloud()->data();

				realm::PointCloud::Ptr sprase = std::make_shared<realm::PointCloud>(point_ids, points);

				frame_stage->setSparseCloud(sprase, false);
			}
		}

		if (_is_master_stage)
		{
			if (!_gnss_base_initialized)
			{
				UTMPose utm_pose = frame->getGnssUtm();

				double gnss_base_lon, gnss_base_lat, gnss_base_alt;
				if (utm2Wgs84Point(utm_pose.zone, utm_pose.band, utm_pose.easting, utm_pose.northing, utm_pose.altitude - 100.0,
					&gnss_base_lon, &gnss_base_lat, &gnss_base_alt))
				{
					_gnss_base = cv::Vec3d(gnss_base_lon, gnss_base_lat, gnss_base_alt);
					_gnss_base_initialized = true;
				}
			}
		}

		if (_stage)
		{
			_stage->addFrame(std::move(frame_stage));
		}
	}

	void StageNode::subImu(const VisualSlamIF::ImuData& imu)
	{
		/*VisualSlamIF::ImuData imu;
		imu.timestamp = msg.header.stamp.sec;
		imu.acceleration.x = msg.linear_acceleration.x;
		imu.acceleration.y = msg.linear_acceleration.y;
		imu.acceleration.z = msg.linear_acceleration.z;
		imu.gyroscope.x = msg.angular_velocity.x;
		imu.gyroscope.y = msg.angular_velocity.y;
		imu.gyroscope.z = msg.angular_velocity.z;*/
		reinterpret_cast<stages::PoseEstimation*>(_stage.get())->queueImuData(imu);
	}

	void StageNode::subOutputPath(const std::string& p_dir_date_time)
	{
		// check if output directory has changed
		if (_dir_date_time != p_dir_date_time)
		{
			// Note: master privilege is not to create folder, but to set the name of the folder
			_dir_date_time = p_dir_date_time;
			if (!io::dirExists(_path_output + "/" + _dir_date_time))
				io::createDir(_path_output + "/" + _dir_date_time);
			_stage->initStagePath(_path_output + "/" + _dir_date_time);

			// Debug info
			LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Received output directory, set to:\n\t" << _path_output + "/" + _dir_date_time;
		}
	}

	void StageNode::pubFrame(const Frame::Ptr& frame, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;

		publisher->pubFrame(frame);
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Published frame.";
	}

	void StageNode::pubPose(const cv::Mat& pose, uint8_t zone, char band, const std::string& topic)
	{
		//std_msgs::Header header;
		//header.stamp = ros::Time::now();

		//// utm
		//geometry_msgs::PoseStamped utm_msg;
		//utm_msg.header = header;
		//utm_msg.header.frame_id = "utm";
		//utm_msg.pose = to_ros::pose(pose);

		//// wgs
		//geometry_msgs::PoseStamped wgs_msg;
		//wgs_msg.header = header;
		//wgs_msg.header.frame_id = "wgs";
		//wgs_msg.pose = to_ros::poseWgs84(pose, zone, band);

		_publisher[topic + "/utm"]->pubPose(pose, zone, band);

		std::vector<cv::Vec3d>* trajectory = &_trajectories[topic];
		trajectory->push_back(cv::Vec3d(
			pose.at<double>(0, 3),
			pose.at<double>(1, 3),
			pose.at<double>(2, 3)));
		pubTrajectory(*trajectory, topic + "/traj");
	}

	void StageNode::pubPointCloud(const PointCloud::Ptr& sparse_cloud, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;

		publisher->pubPointCloud(sparse_cloud);
	}

	void StageNode::pubDepthMap(const cv::Mat& img, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;

		publisher->pubDepthMap(img);
	}

	void StageNode::pubImage(const cv::Mat& img, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;

		publisher->pubImage(img);
	}

	void StageNode::pubMesh(const std::vector<Face>& faces, const std::string& topic)
	{
		std::unique_lock<std::mutex> lock(_mutex_do_shutdown);


		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;


		publisher->pubMesh(faces);
	}

	void StageNode::pubTrajectory(const std::vector<cv::Vec3d>& traj, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];
		if (publisher->getNumSubscribers() == 0)
			return;

		publisher->pubTrajectory(traj);
	}

	void StageNode::pubCvGridMap(const CvGridMap& map, uint8_t zone, char band, const std::string& topic)
	{
		MyPublisher* publisher = _publisher[topic];

		if (publisher->getNumSubscribers() == 0)
			return;


		publisher->pubCvGridMap(map);
	}

	bool StageNode::srvFinish()
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Requesting stage finishCallback...!";
		_stage->requestFinish();
		_stage->join();
		_is_stage_running = false;
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully finished stage!";
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Shutting stage node down...";
		std::unique_lock<std::mutex> lock(_mutex_do_shutdown);
		_do_shutdown = true;
		return true;
	}

	bool StageNode::srvStop()
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Requesting stage stop...!";
		_stage->requestStop();
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully stopped stage!";
		return true;
	}

	bool StageNode::srvResume()
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Requesting stage resume...!";
		_stage->resume();
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully resumed stage!";
		return true;
	}

	bool StageNode::srvReset()
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Requesting stage reset...!";
		_stage->requestReset();
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully reset stage!";
		return true;
	}

	bool StageNode::srvChangeParam(const std::string& para_name,
		const std::string& para_val)
	{
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Changing stage parameter " << para_name
			<< " to value " << para_val << "...";
		if (_stage->changeParam(para_name, para_val))
		{
			LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Successfully changed parameter!";
			return true;
		}
		else
		{
			LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Failed to change parameter!";
			return false;
		}
	}

	void StageNode::readStageSettings()
	{
		// Load stage settings
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Loading stage settings from path:\n\t"
			<< _file_settings_stage.c_str();
		_settings_stage = StageSettingsFactory::load(_type_stage, _file_settings_stage);
		LOG(INFO) << "STAGE_NODE [" << _type_stage << "]: Detected stage type: '"
			<< (*_settings_stage)["type"].toString() << "'";
	}

	void StageNode::readParams()
	{
		std::string str_is_master_stage;
		// Read parameters from launch file
		_node_paras.param("stage/type", _type_stage, std::string("uninitialised"));
		_node_paras.param("stage/master", str_is_master_stage, "false");
		_node_paras.param("stage/output_dir", _path_output, std::string("uninitialised"));
		_node_paras.param("topics/input/frame", _topic_frame_in, std::string("uninitialised"));
		_node_paras.param("topics/input/imu", _topic_imu_in, std::string("uninitialised"));
		_node_paras.param("topics/output", _topic_frame_out, std::string("uninitialised"));
		_node_paras.param("config/id", _id_camera, std::string("uninitialised"));
		_node_paras.param("config/profile", _profile, std::string("uninitialised"));
		_node_paras.param("config/method", _method, std::string("uninitialised"));
		_node_paras.param("config/opt/working_directory", _path_working_directory, std::string("uninitialised"));
		_node_paras.param("config/opt/output_directory", _path_output, std::string("uninitialised"));

		_is_master_stage = str_is_master_stage.compare("true") == 0;

		// Set specific config file paths
		if (_profile == "uninitialised")
			throw(std::invalid_argument("Error: Stage settings profile must be provided in launch file."));
		if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
			throw(std::invalid_argument("Error: Working directory does not exist!"));
	}

	void StageNode::setPaths()
	{
		_path_profile = _path_working_directory + "/profiles/" + _profile;

		if (_path_output == "uninitialised")
			_path_output = _path_working_directory + "/output";

		// Set settings filepaths
		_file_settings_camera = _path_profile + "/camera/calib.yaml";
		_file_settings_imu = _path_profile + "/config/imu.yaml";
		_file_settings_stage = _path_profile + "/" + _type_stage + "/stage_settings.yaml";
		_file_settings_method = _path_profile + "/" + _type_stage + "/method/" + _method + "_settings.yaml";

		if (!io::dirExists(_path_profile))
			throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
		if (!io::dirExists(_path_output))
			io::createDir(_path_output);

		// Master priviliges
		if (_is_master_stage)
		{
			// Create sub directory with timestamp
			_dir_date_time = io::getDateTime();
			if (!io::dirExists(_path_output + "/" + _dir_date_time))
				io::createDir(_path_output + "/" + _dir_date_time);
		}
	}

}