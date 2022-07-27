#ifndef MYREALM_MESHSCENENODE_H
#define MYREALM_MESHSCENENODE_H

#include "MyREALM_Core_Exports.h"

#include <deque>
#include <map>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM
{
	class MeshReceiverThread : public OpenThreads::Thread
	{
	public:
		MeshReceiverThread();
		~MeshReceiverThread();

		virtual int cancel();
		virtual void run();

		void updateMeshGeode(osg::ref_ptr<osg::Vec3Array> vec_arr,
			osg::ref_ptr<osg::Vec4Array> clr_arr,
			osg::ref_ptr<osg::Vec2Array> texture_arr,
			osg::ref_ptr<osg::UIntArray> face_ary,
			osg::ref_ptr<osg::Image> texImg, const std::string& save_osgb_filepath = "");

		bool getMeshGeode(osg::ref_ptr<osg::Geode>& geod);

		void clear();

		inline bool isDirty() const { return _geod_que.size() > 0; }

	protected:
		OpenThreads::Mutex _mutex;
		std::deque<osg::ref_ptr<osg::Geode>> _geod_que;
		size_t _max_que_size;
		bool _done;
	};


	class MeshNodeCallback :public osg::NodeCallback
	{
	public:
		MeshNodeCallback(MeshReceiverThread* p_thread);
		~MeshNodeCallback();

		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override;

	protected:
		MeshReceiverThread* m_thread;
	};


	osg::ref_ptr<osg::Geode> createMeshGeode(
		osg::ref_ptr<osg::Vec3Array> vec_arr,
		osg::ref_ptr<osg::Vec4Array> clr_arr,
		osg::ref_ptr<osg::Vec2Array> texture_arr,
		osg::ref_ptr<osg::UIntArray> face_ary,
		osg::ref_ptr<osg::Image> texImg);

}

#endif  // !MYREALM_MESHSCENENODE_H