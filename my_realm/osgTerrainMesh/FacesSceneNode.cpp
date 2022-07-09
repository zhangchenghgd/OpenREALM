#include "FacesSceneNode.h"
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer> //优化器
#include <osgUtil/Simplifier> //简化几何体
#include <iostream>


namespace MyREALM2
{
	osg::ref_ptr<osg::Geode> createFacesGeode(FacesDrawCallback* clb)
	{
		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec2Array> texture_arr = new osg::Vec2Array();
		
		geom->setVertexArray(vec_arr);
		geom->setTexCoordArray(0, texture_arr);

		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		geom->setColorArray(clr_arr);
		geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);


		osg::ref_ptr<osg::Vec3Array> norm_arr = new osg::Vec3Array;
		norm_arr->push_back(osg::Vec3(0.0, 0.0, 1.0));
		geom->setNormalArray(norm_arr);
		geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

		/*osg::ref_ptr<osg::DrawArrays> drawPts = new osg::DrawArrays(
			osg::PrimitiveSet::TRIANGLES, 0, vec_arr->size());*/
		osg::ref_ptr<osg::DrawElementsUInt> triangles =
			new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
		geom->addPrimitiveSet(triangles.get());
		
		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setDataVariance(osg::Object::DYNAMIC);
		geom->setUpdateCallback(clb);
		geod->addChild(geom);

		//设置纹理
		osg::ref_ptr<osg::Image> texImage = osgDB::readImageFile("D:/rlm_dom.png");
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setImage(texImage);
		tex->setDataVariance(osg::Object::DYNAMIC);

		//geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		//渲染状态
		osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
		stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
		stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		stateset->setMode(GL_BLEND, osg::StateAttribute::ON);

		/*osg::ref_ptr<osg::Material> material = new osg::Material;
		material->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
		material->setShininess(osg::Material::FRONT, 90);
		material->setColorMode(osg::Material::AMBIENT);
		stateset->setAttribute(material.get());*/

		geod->setStateSet(stateset.get());
		return geod;
	}

	FacesReceiverThread::FacesReceiverThread() : OpenThreads::Thread()
	{
		_mesh = std::make_shared<Mesh>();
		/*_lineVertics = new osg::Vec3Array;
		_colors = new osg::Vec4Array;
		_indexs = new osg::UIntArray;*/
	}

	int FacesReceiverThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void FacesReceiverThread::run()
	{
		_update_it = 1;
		_done = false;
		_dirty = true;
		do
		{
			//YieldCurrentThread();
			this->updateMesh();
			this->microSleep(5000000);

		} while (!_done);
	}


	void FacesReceiverThread::updateMesh()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		int rows = 500;
		int cols = 500 * _update_it;

		size_t vert_num = rows * cols;
		size_t face_num = (rows - 1) * (cols - 1) * 2;
		_mesh->resize(vert_num, face_num);

		osg::Vec3Array* verts = _mesh->vertices();
		osg::Vec4Array* clrs = _mesh->colors();
		osg::UIntArray* faces = _mesh->faces();
		osg::Vec2Array* texCoords = _mesh->texCoords();

		// Create and add vertices
		for (size_t r = 0; r < rows; ++r)
		{
			for (size_t c = 0; c < cols; ++c)
			{
				size_t idx = r * cols + c;
				verts->at(idx).set(r * 0.1, c * 0.1, 0.0);// (r % 2) * 5 + (c % 2) * 5);
				//clrs->at(idx).set(1.0 * r / rows, 1.0 * c / cols, 1.0, 1.0);
				clrs->at(idx).set(1., 1., 1.0, 1.0);

				texCoords->at(idx).set(1.0 * c / cols, (1.0 * r / rows));

				if (r < rows - 1 && c < cols - 1)
				{
					size_t v0 = r * cols + c;
					size_t v1 = (r + 1) * cols + c;
					size_t v2 = (r + 1) * cols + c + 1;
					size_t v3 = r * cols + c + 1;

					size_t f1_0 = ((cols - 1) * r + c) * 2 * 3;
					size_t f1_1 = f1_0 + 1;
					size_t f1_2 = f1_0 + 2;
					size_t f2_0 = f1_0 + 3;
					size_t f2_1 = f1_0 + 4;
					size_t f2_2 = f1_0 + 5;
					faces->at(f1_0) = v0;
					faces->at(f1_1) = v1;
					faces->at(f1_2) = v2;
					faces->at(f2_0) = v0;
					faces->at(f2_1) = v2;
					faces->at(f2_2) = v3;
				}

			}
		}
		std::cout << "Iterator : " << _update_it << std::endl;
		++_update_it;
		_dirty = true;
	}

	void FacesReceiverThread::setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr,
		const osg::UIntArray& index_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		/*_lineVertics->assign(arr.begin(), arr.end());
		_colors->assign(color_arr.begin(), color_arr.end());
		_indexs->assign(index_arr.begin(), index_arr.end());*/
		_dirty = true;
	}

	bool FacesReceiverThread::getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr,
		osg::Vec2Array& texCoords_arr,
		osg::UIntArray& index_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_dirty)
		{
			arr.assign(_mesh->vertices()->begin(), _mesh->vertices()->end());
			color_arr.assign(_mesh->colors()->begin(), _mesh->colors()->end());
			index_arr.assign(_mesh->faces()->begin(), _mesh->faces()->end());
			texCoords_arr.assign(_mesh->texCoords()->begin(), _mesh->texCoords()->end());
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
		osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
		osg::Vec2Array* texCoords = dynamic_cast<osg::Vec2Array*>(geom->getTexCoordArray(0));
		osg::ref_ptr<osg::UIntArray> indxs = new  osg::UIntArray;
		if (vertices && colors && m_thread && m_thread->getVerticsAndColors(*vertices, *colors,*texCoords, *indxs))
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


	TileFacesNode::TileFacesNode() :osg::Referenced(),
		m_row(0), m_col(0), m_bound()
	{
		m_facesThread = new FacesReceiverThread;
		m_facesCLB = new  FacesDrawCallback(m_facesThread);
		m_geode = createFacesGeode(m_facesCLB);
	}

	TileFacesNode::~TileFacesNode()
	{
		delete m_facesThread;
	}

	const osg::BoundingBoxd& TileFacesNode::getBoundingBox() const
	{
		return m_bound;
	}

	FacesReceiverThread* TileFacesNode::facesThread()
	{
		return m_facesThread;
	}

	osg::ref_ptr<osg::Geode> TileFacesNode::facesGeode()
	{
		return m_geode;
	}

	osg::ref_ptr<TileFacesNode> TileFacesNode::createTileFacesNode(
		int row, int col, double tile_size)
	{
		osg::ref_ptr<TileFacesNode> node = new TileFacesNode;

		double min_x = col * tile_size;
		double max_x = (col + 1) * tile_size;
		double min_y = row * tile_size;
		double max_y = (row + 1) * tile_size;
		double min_z = -9999;
		double max_z = 9999;

		node->m_bound.set(min_x, min_y, min_z,
			max_x, max_y, max_z);
		node->m_row = row;
		node->m_col = col;

		return node;
	}

}