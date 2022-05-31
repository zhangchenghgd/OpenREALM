#ifndef PROJECT_JONFMVTK_NODE_H
#define PROJECT_JONFMVTK_NODE_H

#include "MyREALM_Core_Exports.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

//#include <realm_ros/conversions.h>
#include <realm_io/realm_import.h>
#include <realm_io/exif_import.h>
#include <realm_io/utilities.h>
#include "MyPublisher.h"
#include "NodeParas.h"
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include "jo_fmv_tk.h"
#include "GPSPoseInter.h"



namespace MyREALM
{

	class MyREALM_Core_API JonFmvTKNode : public OpenThreads::Thread
	{
	public:
		JonFmvTKNode(const NodeParas& node_paras);
		~JonFmvTKNode();

		virtual int cancel();
		virtual void run();

		void pause();
		void resume();

		void receiveJoFmvTKFrame(JoFmvTKFrame* vtk_frame);

	private:
		OpenThreads::Mutex _mutex;
		bool _pause;

		NodeParas _node_paras;

		std::string _id_node;
		std::string _profile;
		std::string _topic_prefix;
		std::string _path_working_directory;
		std::string _path_profile;
		std::string _file_settings_camera;
		std::string _url;
		double _fps;
		int _id_curr_vtk_frame;

		MyPublisher* _pub_frame;
		MyPublisher* _pub_image;

		realm::camera::Pinhole::Ptr _cam;
		
		uint64_t _gps_delay_ms;
		TimeGPSPoseInter _gpsPoseInter;
		uint64_t _next_frame_time;

		jav_vtk* _vtk;
		int _vtk_flag;  // 0: failed, 1:success
		
		void readParams();
		void setPaths();
		void pubFrame(const realm::Frame::Ptr& frame);
	};

	class JavVtkOpenThread :public  OpenThreads::Thread
	{
	public:
		JavVtkOpenThread(jav_vtk* vtk, const std::string& url, 
			int* flag, void* opaque);

		virtual int cancel();

		virtual void run();

	private:
		OpenThreads::Mutex _mutex;
		jav_vtk* _vtk;
		std::string _url;
		void* _opaque;
		int* _flag;
	};

} // namespace MyREALM

#endif //PROJECT_JONFMVTK_NODE_H