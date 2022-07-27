#include "MeshSceneNode.h"
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer> //优化器
#include <osgUtil/Simplifier> //简化几何体
#include <osgUtil/SmoothingVisitor> 


namespace MyREALM
{

	osg::ref_ptr<osg::Geode> createMeshGeode(
		osg::ref_ptr<osg::Vec3Array> vec_arr ,
		osg::ref_ptr<osg::Vec4Array> clr_arr ,
		osg::ref_ptr<osg::Vec2Array> texture_arr,
		osg::ref_ptr<osg::UIntArray> face_ary,
		osg::ref_ptr<osg::Image> texImg)
	{
		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

		geom->setVertexArray(vec_arr);
		geom->setTexCoordArray(0, texture_arr);

		//geom->setColorArray(clr_arr);
		//geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		/*osg::ref_ptr<osg::Vec3Array> norm_arr = new osg::Vec3Array;
		norm_arr->push_back(osg::Vec3(0.0, 0.0, 1.0));
		geom->setNormalArray(norm_arr);
		geom->setNormalBinding(osg::Geometry::BIND_OVERALL);*/

		osg::ref_ptr<osg::DrawElementsUInt> triangles =
			new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

		triangles->assign(face_ary->begin(), face_ary->end());

		geom->addPrimitiveSet(triangles.get());

		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setDataVariance(osg::Object::DYNAMIC);
		geod->addChild(geom);


		osgUtil::SmoothingVisitor::smooth(*geom.get());

		//设置纹理

		//osg::ref_ptr<osg::Image> texImage2 = osgDB::readImageFile("D:/rlm_dom.png");
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setName("DOM");
		tex->setImage(texImg); // texImg
		tex->setDataVariance(osg::Object::DYNAMIC);

		//渲染状态
		osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
		stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
		stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
		stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

		osg::ref_ptr<osg::Material> material = new osg::Material;
		material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.986, 0.986, 0.911, 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(0.986, 0.986, 0.911, 1.0));
		material->setShininess(osg::Material::FRONT, 90);
		material->setColorMode(osg::Material::AMBIENT);
		stateset->setAttribute(material.get());

		geom->setStateSet(stateset.get());


		return geod;
	}

	MeshReceiverThread::MeshReceiverThread() : OpenThreads::Thread()
	{
		_geod_que.resize(0);
		_max_que_size = 1;
	}


	MeshReceiverThread::~MeshReceiverThread()
	{
		this->clear();
	}

	int MeshReceiverThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void MeshReceiverThread::run()
	{
		_done = false;
		do
		{
			YieldCurrentThread();

		} while (!_done);
	}

	void MeshReceiverThread::updateMeshGeode(
		osg::ref_ptr<osg::Vec3Array> vec_arr,
		osg::ref_ptr<osg::Vec4Array> clr_arr,
		osg::ref_ptr<osg::Vec2Array> texture_arr,
		osg::ref_ptr<osg::UIntArray> face_ary,
		osg::ref_ptr<osg::Image> texImg,
		const std::string& save_osgb_filepath)
	{
		osg::ref_ptr<osg::Geode> geod = createMeshGeode(vec_arr, clr_arr, texture_arr,
			face_ary, texImg);

		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_geod_que.size() > _max_que_size)
		{
			_geod_que.erase(_geod_que.begin(), _geod_que.begin() + 1);
		}

		_geod_que.push_back(geod);

		if (save_osgb_filepath.length() > 0)
		{
			osgDB::writeNodeFile(*geod->asNode(), save_osgb_filepath);
		}
	}

	bool MeshReceiverThread::getMeshGeode(osg::ref_ptr<osg::Geode>& geod)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (isDirty())
		{
			geod = _geod_que.front();
			_geod_que.pop_front();
			return true;
		}
		return false;
	}


	void MeshReceiverThread::clear()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_geod_que.size() > 0)
		{
			_geod_que.clear();
		}
	}

	MeshNodeCallback::MeshNodeCallback(MeshReceiverThread* p_thread) :
		osg::NodeCallback(), m_thread(p_thread)
	{
	}

	MeshNodeCallback::~MeshNodeCallback()
	{
	}

	void MeshNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::Group* group = node->asGroup();
		if (!group)
			return;

		osg::ref_ptr<osg::Geode> geod = nullptr;
		if (m_thread->getMeshGeode(geod))
		{
			unsigned int child_num = group->getNumChildren();
			if (child_num > 0)
			{
				group->removeChildren(0, child_num);
			}
			group->addChild(geod);
		}
	}


}