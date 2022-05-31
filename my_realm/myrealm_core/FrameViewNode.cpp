#include "FrameViewNode.h"
#include "MyREALM.h"
#include <glog/logging.h>


namespace MyREALM
{
	FrameViewNode::FrameViewNode(const NodeParas& node_paras) :OpenThreads::Thread(),
		_node_paras(node_paras), m_displayTrackedImageFunc(NULL)
	{
		// Read basic launch file inputs
		readParams();

		// Specify stage
		setPaths();


		_topic_prefix = "/realm/" + _id_camera;

		_topic_tracked = "/realm/" + _id_camera + "/pose_estimation/tracked";

		_sub_tracked = MyRealmSys::get_instance().getOrCreatePublisher(_topic_tracked)
			->registSubscriber(_topic_tracked + "/sub");

		SubImageFun subImageFunc = std::bind(&FrameViewNode::subImage, this, std::placeholders::_1);
		_sub_tracked->bindSubImageFunc(subImageFunc);

	}

	FrameViewNode::~FrameViewNode()
	{
		if (isRunning())
		{
			this->cancel();
		}
	}

	void FrameViewNode::bindDisplayTrackedImageFunc(SubImageFun func)
	{
		m_displayTrackedImageFunc = func;
	}

	int FrameViewNode::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void FrameViewNode::run()
	{
		_done = false;
		_dirty = true;
		do
		{
			YieldCurrentThread();

		} while (!_done);
	}

	void FrameViewNode::readParams()
	{
		_node_paras.param("config/id", _id_node, std::string("uninitialised"));
		_node_paras.param("config/opt/working_directory", _path_working_directory, std::string("uninitialised"));

		_id_camera = _id_node;

		if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
			throw(std::invalid_argument("Error: Working directory does not exist!"));
	}

	void FrameViewNode::readStageSettings()
	{
	}

	void FrameViewNode::setPaths()
	{
	}

	void FrameViewNode::subImage(const cv::Mat& image)
	{
		if (m_displayTrackedImageFunc)
		{
			m_displayTrackedImageFunc(image);
		}
	}

}

