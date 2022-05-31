#ifndef MYREALM_FACESSCENENODE_H
#define MYREALM_FACESSCENENODE_H

#include "MyREALM_Core_Exports.h"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM
{
	class FacesReceiverThread : public OpenThreads::Thread
	{
	public:
		FacesReceiverThread();

		virtual int cancel();
		virtual void run();

		void setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr);

		bool getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr);

	protected:
		OpenThreads::Mutex _mutex;
		osg::ref_ptr<osg::Vec3Array> _lineVertics;
		osg::ref_ptr<osg::Vec4Array> _colors;
		bool _done;
		bool _dirty;
	};


	class FacesDrawCallback :public osg::DrawableUpdateCallback
	{
	public:
		FacesDrawCallback(FacesReceiverThread* p_thread);
		~FacesDrawCallback();

		virtual void update(osg::NodeVisitor* nv, osg::Drawable* drawable) override;

	protected:
		FacesReceiverThread* m_thread;
	};


	osg::ref_ptr<osg::Geode> createFacesGeode(FacesDrawCallback* clb);

}

#endif  // !MYREALM_FACESSCENENODE_H