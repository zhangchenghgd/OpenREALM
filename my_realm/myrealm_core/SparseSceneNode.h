#ifndef MYREALM_SPARSESCENENODE_H
#define MYREALM_SPARSESCENENODE_H

#include "MyREALM_Core_Exports.h"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM
{
	class SparseReceiverThread : public OpenThreads::Thread
	{
	public:
		SparseReceiverThread();

		virtual int cancel();
		virtual void run();

		void setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr);

		bool getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr);

	protected:
		OpenThreads::Mutex _mutex;
		osg::ref_ptr<osg::Vec3Array> _vertics;
		osg::ref_ptr<osg::Vec4Array> _colors;
		bool _done;
		bool _dirty;
	};


	class SparseDrawCallback :public osg::DrawableUpdateCallback
	{
	public:
		SparseDrawCallback(SparseReceiverThread* p_thread);
		~SparseDrawCallback();

		virtual void update(osg::NodeVisitor* nv, osg::Drawable* drawable) override;

	protected:
		SparseReceiverThread* m_thread;
	};


	osg::ref_ptr<osg::Geode> createSparseGeode(const osg::Vec4& color, float point_size,
		SparseDrawCallback* clb);

}

#endif  // !MYREALM_SPARSESCENENODE_H