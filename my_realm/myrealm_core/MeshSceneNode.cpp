#include "MeshSceneNode.h"
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>
#include <osgDB/ReadFile>
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

		osg::ref_ptr<osg::Image> texImage2 = osgDB::readImageFile("D:/rlm_dom.png");
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setName("DOM");
		tex->setImage(texImage2); // texImg
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

	osg::ref_ptr<osg::Geode> createFacesGeode(FacesDrawCallback* clb)
	{
		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		osg::ref_ptr<osg::Vec2Array> texture_arr = new osg::Vec2Array();

		/*vec_arr->push_back(osg::Vec3(0,0,0));
		vec_arr->push_back(osg::Vec3(100,0,0));
		vec_arr->push_back(osg::Vec3(100,100,0));
		clr_arr->push_back(osg::Vec4(1, 0, 0,1));
		clr_arr->push_back(osg::Vec4(0, 1, 0,1));
		clr_arr->push_back(osg::Vec4(0, 0, 1,1));
		texture_arr->push_back(osg::Vec2(0, 0));
		texture_arr->push_back(osg::Vec2(1, 0));
		texture_arr->push_back(osg::Vec2(1, 1));*/


		
		geom->setVertexArray(vec_arr);
		geom->setTexCoordArray(0, texture_arr);

		//geom->setColorArray(clr_arr);
		//geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);


		osg::ref_ptr<osg::Vec3Array> norm_arr = new osg::Vec3Array;
		norm_arr->push_back(osg::Vec3(0.0, 0.0, 1.0));
		geom->setNormalArray(norm_arr);
		geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

		/*osg::ref_ptr<osg::DrawArrays> drawPts = new osg::DrawArrays(
			osg::PrimitiveSet::TRIANGLES, 0, vec_arr->size());*/
		osg::ref_ptr<osg::DrawElementsUInt> triangles =
			new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

		/*osg::ref_ptr<osg::UIntArray> iddx_ary = new osg::UIntArray;
		iddx_ary->push_back(0);
		iddx_ary->push_back(1);
		iddx_ary->push_back(2);

		triangles->assign(iddx_ary->begin(), iddx_ary->end());*/

		geom->addPrimitiveSet(triangles.get());


		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setDataVariance(osg::Object::DYNAMIC);
		geom->setUpdateCallback(clb);
		geod->addChild(geom);

		//设置纹理
		osg::ref_ptr<osg::Image> texImage = nullptr;// osgDB::readImageFile("D:/ortho_iter.png");
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setName("DOM");
		tex->setImage(texImage);
		tex->setDataVariance(osg::Object::DYNAMIC);

		//渲染状态
		osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
		stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

		osg::ref_ptr<osg::Material> material = new osg::Material;
		material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.986, 0.986, 0.811, 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
		material->setShininess(osg::Material::FRONT, 90);
		material->setColorMode(osg::Material::AMBIENT);
		stateset->setAttribute(material.get());

		geod->setStateSet(stateset.get());

		return geod;
	}

	FacesReceiverThread::FacesReceiverThread() : OpenThreads::Thread()
	{
		_lineVertics = new osg::Vec3Array;
		_colors = new osg::Vec4Array;
		_texCoords = new osg::Vec2Array;
		_indexs = new osg::UIntArray;
	}

	int FacesReceiverThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void FacesReceiverThread::run()
	{
		_done = false;
		_dirty = true;
		do
		{
			YieldCurrentThread();

		} while (!_done);
	}


	void FacesReceiverThread::setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr,
		const osg::Vec2Array& tex_coords,
		const osg::UIntArray& index_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		_lineVertics->assign(arr.begin(), arr.end());
		_colors->assign(color_arr.begin(), color_arr.end());
		_texCoords->assign(tex_coords.begin(), tex_coords.end());
		_indexs->assign(index_arr.begin(), index_arr.end());
		_dirty = true;
	}

	bool FacesReceiverThread::getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr,
		osg::Vec2Array& tex_coords,
		osg::UIntArray& index_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_dirty)
		{
			
			arr.assign(_lineVertics->begin(), _lineVertics->end());
			color_arr.assign(_colors->begin(), _colors->end());
			tex_coords.assign(_texCoords->begin(), _texCoords->end());
			index_arr.assign(_indexs->begin(), _indexs->end());
			_dirty = false;
			return true;
		}
		return false;
	}

	FacesDrawCallback::FacesDrawCallback(FacesReceiverThread* p_thread) :
		osg::DrawableUpdateCallback(), m_thread(p_thread)
	{
	}

	FacesDrawCallback::~FacesDrawCallback()
	{
	}

	//void update(NodeVisitor *, Drawable *)虚函数，重构后可以实时获取和操作几何体
	//参数：系统内置的访问器对象，用于遍历节点树并找到回调对象的位置；回调所在几何体的对象指针，以便对其进行操作
	void FacesDrawCallback::update(osg::NodeVisitor* nv, osg::Drawable* drawable)
	{
		//dynamic_cast <type-id> (expression):把expression转换成type-id类型的对象, type-id必须是类的指针、类的引用或者void*, expression要与Type-id相同
		//dynamic_cast运算符将一个基类对象指针(或引用)cast到继承类指针，dynamic_cast会根据基类指针是否真正指向继承类指针来做相应处理, 它可以在执行期决定真正的类型
		//dynamic_cast主要用于类层次间的上行转换(和static_cast效果一样)和下行转换(具有类型检查的功能，比static_cast更安全)，还可以用于类之间的交叉转换
		osg::Geometry* geom = dynamic_cast<osg::Geometry*>(drawable);
		if (!geom)
			return;

		/*手动更新：获取数组
		然后在循环中使迭代器依次指向容器中的下一个元素（可以将迭代器看成一个指针，但实际上C++中指针是一种迭代器，迭代器不仅仅是指针），将下一个元素值赋给上一个元素
		*/

		osg::DrawElementsUInt* triangles = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0)->getDrawElements());

		//获取顶点数组，vertices是一个数组指针，即是一个指向顶点数组的指针
		osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		osg::Vec2Array* texCoords = dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(0));
		osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
		osg::ref_ptr<osg::UIntArray> indxs = new  osg::UIntArray;
		if (vertices && colors && m_thread && m_thread->getVerticsAndColors(*vertices, *colors, *texCoords, *indxs))
		{
			triangles->assign(indxs->begin(), indxs->end());
			geom->setPrimitiveSet(0, triangles);

			//使用dirty()将修改结果通知给VBO对象
			vertices->dirty();
			colors->dirty();
			texCoords->dirty();
			triangles->dirty();
		}
	}


}