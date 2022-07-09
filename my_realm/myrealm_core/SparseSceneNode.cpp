#include "SparseSceneNode.h"
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>


namespace MyREALM
{
	osg::ref_ptr<osg::Geode> createSparseGeode(const osg::Vec4& color,
		float point_size, SparseDrawCallback* clb)
	{
		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;

		geom->setVertexArray(vec_arr);

		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		clr_arr->push_back(color);
		geom->setColorArray(clr_arr);
		geom->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::DrawArrays> drawPts = new osg::DrawArrays(
			osg::PrimitiveSet::POINTS, 0, vec_arr->size());
		geom->addPrimitiveSet(drawPts);


		osg::StateSet* ss = geom->getOrCreateStateSet();
		osg::ref_ptr<osg::Point> line_w = new osg::Point(point_size);
		ss->setAttribute(line_w, osg::StateAttribute::ON);
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setDataVariance(osg::Object::DYNAMIC);
		geom->setUpdateCallback(clb);

		geod->addChild(geom);
		return geod;
	}

	SparseReceiverThread::SparseReceiverThread() : OpenThreads::Thread()
	{
		_vertics = new osg::Vec3Array;
		_colors = new osg::Vec4Array;
	}

	int SparseReceiverThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void SparseReceiverThread::run()
	{
		_done = false;
		_dirty = true;
		do
		{
			YieldCurrentThread();

		} while (!_done);
	}


	void SparseReceiverThread::setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		_vertics->assign(arr.begin(), arr.end());
		_colors->assign(color_arr.begin(), color_arr.end());
		_dirty = true;
	}

	bool SparseReceiverThread::getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_dirty)
		{
			arr.assign(_vertics->begin(), _vertics->end());
			color_arr.assign(_colors->begin(), _colors->end());
			_dirty = false;
			return true;
		}
		return false;
	}

	SparseDrawCallback::SparseDrawCallback(SparseReceiverThread* p_thread) :
		osg::DrawableUpdateCallback(), m_thread(p_thread)
	{
	}

	SparseDrawCallback::~SparseDrawCallback()
	{
	}

	//void update(NodeVisitor *, Drawable *)虚函数，重构后可以实时获取和操作几何体
	//参数：系统内置的访问器对象，用于遍历节点树并找到回调对象的位置；回调所在几何体的对象指针，以便对其进行操作
	void SparseDrawCallback::update(osg::NodeVisitor* nv, osg::Drawable* drawable)
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

		osg::ref_ptr<osg::DrawArrays> drawArr = dynamic_cast<osg::DrawArrays*>(geom->getPrimitiveSet(0));
		if (!drawArr.valid())
		{
			return;
		}


		//获取顶点数组，vertices是一个数组指针，即是一个指向顶点数组的指针
		osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(geom->getColorArray());
		osg::ref_ptr<osg::Vec3Array> vertices_add = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> colors_add = new osg::Vec4Array;
		if (vertices && m_thread && m_thread->getVerticsAndColors(*vertices_add.get(), *colors_add.get()))
		{
			vertices->insert(vertices->end(), vertices_add->begin(), vertices_add->end());
			/*colors->insert(colors->end(), colors_add->begin(), colors_add->end());

			if (colors->size() == vertices->size())
			{
				geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			}
			else
			{
				geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			}*/

			drawArr->setCount(vertices->size());

			/*osg::ref_ptr<osg::DrawArrays> drawPts = new osg::DrawArrays(
				osg::PrimitiveSet::POINTS, 0, vertices->size());*/
			geom->setPrimitiveSet(0, drawArr.get());

			//使用dirty()将修改结果通知给VBO对象
			vertices->dirty();
			//colors->dirty();
			drawArr->dirty();
		}
	}


}