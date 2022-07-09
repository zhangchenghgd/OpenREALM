#include "JonFmvTKNode.h"
#include "MyREALM.h"
#include <OpenThreads/ScopedLock>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <chrono>
#include <thread>
#include <map>

using namespace std::chrono;

#define USE_SYNC 0

using namespace realm;

namespace MyREALM
{

	static void decoded_frame_callback(JoFmvTKFrame* frame, void* opaque);
	static void muxed_data_callback(JoFmvTKMuxedData* muxed_data, void* opaque);

	JonFmvTKNode::JonFmvTKNode(const NodeParas& node_paras)
		:OpenThreads::Thread(), 
		_node_paras(node_paras),
		_fps(0.0),
		_cam(nullptr),
		_id_curr_vtk_frame(0),
		_gps_delay_ms(0),
		_next_frame_time(0),
		_requireDisconnect(false),
		_vtk(nullptr),
		_ConnectCLB(NULL)
	{
		readParams();
		setPaths();

		LOG(INFO) << "Start";

		if (io::fileExists(_file_settings_camera))
		{
			_cam = std::make_shared<camera::Pinhole>(
				io::loadCameraFromYaml(_file_settings_camera));
		}
		else
		{
			throw(std::invalid_argument(
				"Error loading camera file: Provided path does not exist."));
		}
	   

		LOG(INFO) <<
			"JonFmvTKNode Node: Successfully loaded camera: "
			<< "\n\tcx = " << _cam->cx()
			<< "\n\tcy = " << _cam->cy()
			<< "\n\tfx = " << _cam->fx()
			<< "\n\tfy = " << _cam->fy()
			<< "\n\tk1 = " << _cam->k1()
			<< "\n\tk2 = " << _cam->k2()
			<< "\n\tp1 = " << _cam->p1()
			<< "\n\tp2 = " << _cam->p2()
			<< "\n\tk3 = " << _cam->k3();

		// ROS related inits
		_topic_prefix = "/realm/" + _id_node;

		_pub_frame = MyRealmSys::get_instance().getOrCreatePublisher(_topic_prefix + "/input");
		
	}

	JonFmvTKNode::~JonFmvTKNode()
	{
		if (_vtk)
		{
			_requireDisconnect = true;

			while (isRunning()) YieldCurrentThread();
		}
	}

	void JonFmvTKNode::readParams()
	{
		std::string _fps_str;
		std::string _gps_dealy_ms_str = "0";

		_node_paras.param("config/id", _id_node, std::string("uninitialised"));
		_node_paras.param("config/input", _url, std::string("uninitialised"));
		_node_paras.param("config/rate", _fps_str, "0.0");
		_node_paras.param("config/profile", _profile, std::string("uninitialised"));
		_node_paras.param("config/gps_delay_ms", _gps_dealy_ms_str, "0");
		_node_paras.param("config/opt/working_directory",
			_path_working_directory, std::string("uninitialised"));
		

		_fps = atof(_fps_str.c_str());
		_gps_delay_ms = atoi(_gps_dealy_ms_str.c_str());


		if (_fps < 0.01)
		{
			throw(std::invalid_argument(
				"Error reading JonFmvTKNode parameters:"\
				" Frame rate is too low!"));
		}
		if (_path_working_directory != "uninitialised" 
			&& !io::dirExists(_path_working_directory))
		{
			throw(std::invalid_argument(
				"Error: Working directory does not exist!"));
		}
	}

	void JonFmvTKNode::setPaths()
	{
		_path_profile = _path_working_directory 
			+ "/profiles/" + _profile;

		_file_settings_camera = _path_profile + "/camera/calib.yaml";

		if (!io::dirExists(_path_profile))
		{
			throw(std::invalid_argument("Error: Config folder path '"
				+ _path_profile + "' does not exist!"));
		}
	}

	void JonFmvTKNode::pubFrame(const Frame::Ptr& frame)
	{
		_pub_frame->pubFrame(frame);
	}

	void JonFmvTKNode::updateConnectStatus(ConnectStatus status)
	{
		_connectStatus = status;
		if (_ConnectCLB)
		{
			_ConnectCLB((int)status);
		}
	}

	int JonFmvTKNode::cancel()
	{
		if (_vtk)
		{
			_requireDisconnect = true;
			while (isRunning()) YieldCurrentThread();
		}
		return 0;
	}

	void JonFmvTKNode::run()
	{
		_id_curr_vtk_frame = 0;
		_pause = false;
		_requireDisconnect = false;

		// 创建一个实例
		_vtk = jav_vtk_create();

		JavVtkOpenThread* vtkOpenTd = 
			new JavVtkOpenThread(_vtk, _url, this);

		vtkOpenTd->startThread();

		do
		{
			if (_requireDisconnect)
			{
				_requireDisconnect = false;
				break;
			}
			else
			{
				OpenThreads::Thread::microSleep(10000);
			}
		} while (_connectStatus == Connecting || _connectStatus == ConnectSuccess);

		// 结束处理流程，如果open还未返回，此时会返回
		jav_vtk_close(_vtk);
		

		if (vtkOpenTd->isRunning())
		{
			vtkOpenTd->join();
		}

		// 清理所有资源
		jav_vtk_destroy(_vtk);

		delete vtkOpenTd;

		_vtk = nullptr;

		updateConnectStatus(Disconnect);
	}

	void JonFmvTKNode::pause()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		_pause = true;
	}

	void JonFmvTKNode::resume()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		_pause = false;
	}

	void JonFmvTKNode::receiveJoFmvTKFrame(JoFmvTKFrame* vtk_frame)
	{
		uint32_t frame_id = _id_curr_vtk_frame++;

		if (_pause) return;

		int w = vtk_frame->width;
		int h = vtk_frame->height;
		
		if (h != _cam->height() || 
			w != _cam->width() || 
			vtk_frame->data == NULL)
		{
			return;
		}
		
		uint64_t cur_time = vtk_frame->current_time;
		double gps_lon = vtk_frame->meta_data_basic.CarrierVehiclePosInfo_p.CarrierVehicleLon * 0.0000001;
		double gps_lat = vtk_frame->meta_data_basic.CarrierVehiclePosInfo_p.CarrierVehicleLat * 0.0000001;
		double gps_alt = vtk_frame->meta_data_basic.CarrierVehiclePosInfo_p.CarrierVehicleHMSL * 0.01;
		double heading_angle = vtk_frame->meta_data_basic.CarrierVehiclePosInfo_p.CarrierVehicleHeadingAngle * 0.0001
			* 180.0 / M_PI;
		
		_gpsPoseInter.putGPS(cur_time, cv::Vec3d(gps_lon, gps_lat, gps_alt));
		_gpsPoseInter.putHeadingAngle(cur_time, heading_angle);

		if (cur_time < _next_frame_time)
		{
			return;
		}

		uint64_t delay_timestamp = cur_time - _gps_delay_ms;

		cv::Vec3d frame_gps;
		double frame_heading;
		bool has_gps_h = _gpsPoseInter.interGPS(delay_timestamp, frame_gps) && 
			_gpsPoseInter.interHeadingAngle(delay_timestamp, frame_heading);

		if(!has_gps_h)
		{
			return;
		}

		WGSPose wgs{ 0 };
		wgs.longitude = frame_gps[0];
		wgs.latitude = frame_gps[1];
		wgs.altitude = frame_gps[2];
		wgs.heading = frame_heading;

		cv::Mat img_src = cv::Mat(h, w, CV_8UC3, vtk_frame->data);
		cv::Mat img = img_src.clone();

		realm::camera::Pinhole::Ptr frame_cam = 
			std::make_shared<realm::camera::Pinhole>(*_cam.get());


		/*========== ESSENTIAL KEYS ==========*/
		

		UTMPose utm = gis::convertToUTM(wgs);

		/*========== OPTIONAL KEYS ==========*/
		uint64_t timestamp_val = vtk_frame->current_time;


		Frame::Ptr frame = std::make_shared<Frame>(_id_node, frame_id, timestamp_val,
			img, utm, frame_cam, realm::io::computeOrientationFromHeading(utm.heading));


		pubFrame(frame);

		_next_frame_time = cur_time + (int)(1000 / _fps);
	
	}

	void JonFmvTKNode::bindJoUAVConnectCLB(JoUAVConnectCLB clb)
	{
		_ConnectCLB = clb;
	}


	static void decoded_frame_callback(JoFmvTKFrame* frame, void* opaque)
	{
		if (frame) 
		{
			if (opaque)
			{
				JonFmvTKNode* jonNode =(JonFmvTKNode*)(opaque);
				if (jonNode)
				{
					jonNode->receiveJoFmvTKFrame(frame);
				}
			}

			jav_vtk_frame_free(frame);// frame使用完后需要手动删除
		}
	}

	static void muxed_data_callback(JoFmvTKMuxedData* muxed_data, void* opaque)
	{
		if (muxed_data) {
			jav_vtk_muxed_data_free(muxed_data);// muxed_data使用完后需要手动删除
		}
	}


	JavVtkOpenThread::JavVtkOpenThread(jav_vtk* vtk, 
		const std::string& url, JonFmvTKNode* opaque)
		:OpenThreads::Thread(),
		_vtk(vtk), _url(url), _opaque(opaque)
	{

	}

	int JavVtkOpenThread::cancel()
	{
		return 0;
	}

	void JavVtkOpenThread::run()
	{
		_opaque->updateConnectStatus(JonFmvTKNode::Connecting);
		auto result = jav_vtk_open_cb(
			_vtk,                    // 实例句柄
			_url.c_str(),            // 视频流地址
			_opaque,                 // 透传指针
			nullptr,                 // 元数据回调
			nullptr,                 // 视频裸码回调
			decoded_frame_callback,  // 解码后的图像输入回调
			muxed_data_callback,     // 复用后的数据回调
			MuxedDataType_TS);       // 复用数据格式，如果muxed_data_callback为空，则忽略
	
		if (result != 0) {
			std::cout << "fmv tk open cb failed" << std::endl;
			_opaque->updateConnectStatus(JonFmvTKNode::ConnectFailed);
		}
		else {
			_opaque->updateConnectStatus(JonFmvTKNode::ConnectSuccess);
		}
	}

}