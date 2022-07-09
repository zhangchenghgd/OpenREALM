#ifndef MYREALM_TRAJSCENENODE_H
#define MYREALM_TRAJSCENENODE_H

#include "MyREALM_Core_Exports.h"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM
{
	/**
	 * @brief 轨迹线接收集合顶点线程.
	 */
	class TrajReceiverThread : public OpenThreads::Thread
	{
	public:
		TrajReceiverThread();

		virtual int cancel();
		virtual void run();

		void setVertics(const osg::Vec3Array& arr);

		bool getVertics(osg::Vec3Array& arr);

	protected:
		OpenThreads::Mutex _mutex;
		osg::ref_ptr<osg::Vec3Array> _lineVertics;      // 轨迹线顶点
		bool _done;
		bool _dirty;
	};


	class TrajDrawCallback :public osg::DrawableUpdateCallback
	{
	public:
		TrajDrawCallback(TrajReceiverThread* p_thread);
		~TrajDrawCallback();

		virtual void update(osg::NodeVisitor* nv, osg::Drawable* drawable) override;

	protected:
		TrajReceiverThread* m_thread;
	};

	/**
	 * @brief 创建轨迹线叶节点.
	 */
	osg::ref_ptr<osg::Geode> createTrajGeode(const osg::Vec4& color, float line_width,
		TrajDrawCallback* clb);

}

#endif  // !MYREALM_TRAJSCENENODE_H