#ifndef MYREALM_MESHSCENENODE_H
#define MYREALM_MESHSCENENODE_H

#include "MyREALM_Core_Exports.h"

#include <map>
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

		void setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr,
			const osg::Vec2Array& tex_coords,
			const osg::UIntArray& index_arr);

		bool getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr,
			osg::Vec2Array& tex_coords,
			osg::UIntArray& index_arr);

	protected:
		OpenThreads::Mutex _mutex;
		osg::ref_ptr<osg::Vec3Array> _lineVertics;
		osg::ref_ptr<osg::Vec4Array> _colors;
		osg::ref_ptr<osg::Vec2Array> _texCoords;
		osg::ref_ptr<osg::UIntArray> _indexs;
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

	osg::ref_ptr<osg::Geode> createMeshGeode(
		osg::ref_ptr<osg::Vec3Array> vec_arr,
		osg::ref_ptr<osg::Vec4Array> clr_arr,
		osg::ref_ptr<osg::Vec2Array> texture_arr,
		osg::ref_ptr<osg::UIntArray> face_ary,
		osg::ref_ptr<osg::Image> texImg);

}

#endif  // !MYREALM_MESHSCENENODE_H