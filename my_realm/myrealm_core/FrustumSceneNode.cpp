﻿#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "FrustumSceneNode.h"

namespace MyREALM
{


	Frustum::Frustum()
		: z_near(-1.),
		z_far(-1.)
	{
		
	}

	Frustum::Frustum
	(
		const int w,
		const int h,
		const Eigen::Matrix<double, 3, 3>& K,
		const Eigen::Matrix<double, 3, 3>& R,
		const Eigen::Vector3d& C
	): z_near(-1.),z_far(-1.)
	{
		const Eigen::Matrix<double, 3, 3> Kinv = K.inverse(); 
		const Eigen::Matrix<double, 3, 3> Rt = R.transpose();

		// Definition of the frustum with the supporting points
		cones[0] = C;
		cones[1] = Rt * ((Kinv * Eigen::Vector3d(0, 0, 1.0))) + C;
		cones[2] = Rt * ((Kinv * Eigen::Vector3d(w, 0, 1.0))) + C;
		cones[3] = Rt * ((Kinv * Eigen::Vector3d(w, h, 1.0))) + C;
		cones[4] = Rt * ((Kinv * Eigen::Vector3d(0, h, 1.0))) + C;

		// supporting point for drawing is a normalized cone, since infinity cannot be represented
		points = std::vector<Eigen::Vector3d>(&cones[0], &cones[0] + 5);
	}

	Frustum::Frustum
	(
		const int w,
		const int h,
		const Eigen::Matrix<double, 3, 3>& K,
		const Eigen::Matrix<double, 3, 3>& R,
		const Eigen::Vector3d& C,
		const double zFar
	) : z_near(-1.), z_far(zFar)
	{
		const Eigen::Matrix<double, 3, 3> Kinv = K.inverse();
		const Eigen::Matrix<double, 3, 3> Rt = R.transpose();

		// Definition of the frustum with the supporting points
		cones[0] = C;
		cones[1] = Rt * (z_far * (Kinv * Eigen::Vector3d(0, 0, 1.0))) + C;
		cones[2] = Rt * (z_far * (Kinv * Eigen::Vector3d(w, 0, 1.0))) + C;
		cones[3] = Rt * (z_far * (Kinv * Eigen::Vector3d(w, h, 1.0))) + C;
		cones[4] = Rt * (z_far * (Kinv * Eigen::Vector3d(0, h, 1.0))) + C;

		// supporting point for drawing is a normalized cone, since infinity cannot be represented
		points = std::vector<Eigen::Vector3d>(&cones[0], &cones[0] + 5);
	}

	

	const std::vector<Eigen::Vector3d>& Frustum::frustum_points() const
	{
		return points;
	}

	osg::ref_ptr<osg::Geode> createFrustumNode(
		const realm::camera::Pinhole::ConstPtr& cam, double zFar,
		const osg::Vec4& color)
	{

		int w = cam->width();
		int h = cam->height();

		Eigen::Matrix<double, 3, 3> K;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				K(r, c) = cam->K().at<double>(r, c);
			}
		}

		Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();
		Eigen::Vector3d C(0,0,0);


		const Frustum frustum(w, h, K,
			R, C, zFar);


		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		Eigen::Vector3d plane_cent = (frustum.cones[1] + frustum.cones[3]) / 2.0;
		//openMVG::Vec3 plane_cent2 = (frustum.cones[2] + frustum.cones[4]) / 2.0;


		Eigen::Vector3d plane_t = (frustum.cones[1] + frustum.cones[2]) / 2.0;
		Eigen::Vector3d plane_b = (frustum.cones[3] + frustum.cones[4]) / 2.0;
		Eigen::Vector3d plane_l = (frustum.cones[1] + frustum.cones[4]) / 2.0;
		Eigen::Vector3d plane_r = (frustum.cones[2] + frustum.cones[3]) / 2.0;


		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		vec_arr->push_back(osg::Vec3(frustum.cones[0].x(), frustum.cones[0].y(), frustum.cones[0].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[1].x(), frustum.cones[1].y(), frustum.cones[1].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[2].x(), frustum.cones[2].y(), frustum.cones[2].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[3].x(), frustum.cones[3].y(), frustum.cones[3].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[4].x(), frustum.cones[4].y(), frustum.cones[4].z()));
		vec_arr->push_back(osg::Vec3(plane_t.x(), plane_t.y(), plane_t.z()));
		vec_arr->push_back(osg::Vec3(plane_b.x(), plane_b.y(), plane_b.z()));
		vec_arr->push_back(osg::Vec3(plane_l.x(), plane_l.y(), plane_l.z()));
		vec_arr->push_back(osg::Vec3(plane_r.x(), plane_r.y(), plane_r.z()));
		vec_arr->push_back(osg::Vec3(plane_cent.x(), plane_cent.y(), plane_cent.z()));

		osg::ref_ptr<osg::Vec2Array> tex_arr = new osg::Vec2Array;
		tex_arr->push_back(osg::Vec2(0, 0));
		tex_arr->push_back(osg::Vec2(1, 0));
		tex_arr->push_back(osg::Vec2(1, 1));
		tex_arr->push_back(osg::Vec2(0, 1));

		osg::ref_ptr<osg::Geometry> geom_outline = new osg::Geometry;
		osg::ref_ptr<osg::Geometry> geom_faces = new osg::Geometry;
		geom_outline->setVertexArray(vec_arr);
		geom_faces->setVertexArray(vec_arr);

		osg::ref_ptr<osg::Vec4Array> line_clr_arr = new osg::Vec4Array;
		line_clr_arr->push_back(color);
		geom_outline->setColorArray(line_clr_arr);
		geom_outline->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::Vec4Array> face_clr_arr = new osg::Vec4Array;
		face_clr_arr->push_back(osg::Vec4(1.0, 0.0, 0.0, 0.5)); // 红色
		face_clr_arr->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.5)); // 绿色
		face_clr_arr->push_back(osg::Vec4(0.0, 0.0, 1.0, 0.5)); // 蓝色
		face_clr_arr->push_back(osg::Vec4(1.0, 1.0, 0.0, 0.5)); // 黄色
		face_clr_arr->push_back(osg::Vec4(1.0, 0.0, 1.0, 0.5)); // 紫色
		face_clr_arr->push_back(osg::Vec4(0.0, 1.0, 1.0, 0.5)); // 青色
		geom_faces->setColorArray(face_clr_arr);
		geom_faces->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::DrawElementsUInt> outline0 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP, 4);
		outline0->at(0) = 1;
		outline0->at(1) = 2;
		outline0->at(2) = 3;
		outline0->at(3) = 4;

		osg::ref_ptr<osg::DrawElementsUInt> outline1 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP, 3);
		outline1->at(0) = 0;
		outline1->at(1) = 1;
		outline1->at(2) = 2;

		osg::ref_ptr<osg::DrawElementsUInt> outline2 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP, 3);
		outline2->at(0) = 0;
		outline2->at(1) = 2;
		outline2->at(2) = 3;

		osg::ref_ptr<osg::DrawElementsUInt> outline3 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP, 3);
		outline3->at(0) = 0;
		outline3->at(1) = 3;
		outline3->at(2) = 4;

		osg::ref_ptr<osg::DrawElementsUInt> outline4 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_LOOP, 3);
		outline4->at(0) = 0;
		outline4->at(1) = 4;
		outline4->at(2) = 1;

		/*osg::ref_ptr<osg::DrawElementsUInt> outline5 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);
		outline5->at(0) = 0;
		outline5->at(1) = 5;*/

		osg::ref_ptr<osg::DrawElementsUInt> outline5 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);


		osg::ref_ptr<osg::DrawElementsUInt> outline6 = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);

		outline5->at(0) = 1;
		outline5->at(1) = 3;
		outline6->at(0) = 2;
		outline6->at(1) = 4;

		osg::ref_ptr<osg::DrawElementsUInt> faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);
		faces->at(0) = 0;
		faces->at(1) = 1;
		faces->at(2) = 2;
		faces->at(3) = 3;
		faces->at(4) = 4;
		faces->at(5) = 1;

		geom_faces->addPrimitiveSet(faces);

		geom_outline->addPrimitiveSet(outline0);
		geom_outline->addPrimitiveSet(outline1);
		geom_outline->addPrimitiveSet(outline2);
		geom_outline->addPrimitiveSet(outline3);
		geom_outline->addPrimitiveSet(outline4);
		//geom_outline->addPrimitiveSet(outline5);
		//geom_outline->addPrimitiveSet(outline6);

		osg::ref_ptr<osg::LineWidth> line_w = new osg::LineWidth(1.0f);
		geom_outline->getOrCreateStateSet()->setAttribute(line_w, osg::StateAttribute::ON);
		geom_outline->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		geom_faces->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		geom_faces->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		geom_faces->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		//geod->addDrawable(geom_faces);
		geod->addDrawable(geom_outline);


#if 0
		{
			// 添加坐标轴 X-red Y-green Z-blue
			osg::ref_ptr<osg::Geometry> axis_geom = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> vec_arr2 = new osg::Vec3Array;
			vec_arr2->push_back(osg::Vec3(0, 0, 0)* zFar * 0.5);
			vec_arr2->push_back(osg::Vec3(1, 0, 0)* zFar * 0.5);
			vec_arr2->push_back(osg::Vec3(0, 1, 0)* zFar * 0.5);
			vec_arr2->push_back(osg::Vec3(0, 0, 1)* zFar * 0.5);
			axis_geom->setVertexArray(vec_arr2);

			osg::ref_ptr<osg::Vec4Array> line_clr_arr2 = new osg::Vec4Array;
			line_clr_arr2->push_back(osg::Vec4(1., 0., 0., 1.));
			line_clr_arr2->push_back(osg::Vec4(0., 1., 0., 1.));
			line_clr_arr2->push_back(osg::Vec4(0., 0., 1., 1.));
			axis_geom->setColorArray(line_clr_arr2);
			axis_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

			osg::ref_ptr<osg::DrawElementsUInt> axis_x = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);
			axis_x->at(0) = 0;
			axis_x->at(1) = 1;

			osg::ref_ptr<osg::DrawElementsUInt> axis_y = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);
			axis_y->at(0) = 0;
			axis_y->at(1) = 2;

			osg::ref_ptr<osg::DrawElementsUInt> axis_z = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 2);
			axis_z->at(0) = 0;
			axis_z->at(1) = 3;
			axis_geom->addPrimitiveSet(axis_x);
			axis_geom->addPrimitiveSet(axis_y);
			axis_geom->addPrimitiveSet(axis_z);

			osg::ref_ptr<osg::LineWidth> line_w2 = new osg::LineWidth(2.0f);
			axis_geom->getOrCreateStateSet()->setAttribute(line_w2, osg::StateAttribute::ON);
			axis_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


			geod->addChild(axis_geom);
		}


#endif // 0


		return geod;
	}


	osg::ref_ptr<osg::Geode> generateARImageNode(
		const realm::camera::Pinhole::ConstPtr& cam, double zFar,
		ArImageUpdateCallback* clb)
	{
		int w = cam->width();
		int h = cam->height();

		Eigen::Matrix<double, 3, 3> K;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				K(r, c) = cam->K().at<double>(r, c);
			}
		}

		Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();
		Eigen::Vector3d C(0, 0, 0);

		const Frustum frustum(w, h, K,
			R, C, zFar);

		osg::ref_ptr<osg::Geode> geod = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

		osg::ref_ptr<osg::Vec3Array> vec_arr = new osg::Vec3Array;
		vec_arr->push_back(osg::Vec3(frustum.cones[4].x(), frustum.cones[4].y(), frustum.cones[4].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[3].x(), frustum.cones[3].y(), frustum.cones[3].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[2].x(), frustum.cones[2].y(), frustum.cones[2].z()));
		vec_arr->push_back(osg::Vec3(frustum.cones[1].x(), frustum.cones[1].y(), frustum.cones[1].z()));
		geom->setVertexArray(vec_arr.get());

		osg::ref_ptr<osg::Vec4Array> clr_arr = new osg::Vec4Array;
		clr_arr->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
		geom->setColorArray(clr_arr);
		geom->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::Vec2Array> tex_arr = new osg::Vec2Array;
		tex_arr->push_back(osg::Vec2(0.0, 0.0));
		tex_arr->push_back(osg::Vec2(1.0, 0.0));
		tex_arr->push_back(osg::Vec2(1.0, 1.0));
		tex_arr->push_back(osg::Vec2(0.0, 1.0));
		geom->setTexCoordArray(0, tex_arr.get());

		
		osg::ref_ptr<osg::Vec3Array> norm_arr = new osg::Vec3Array;
		norm_arr->push_back(osg::Vec3(0.0, 0.0, 1.0));
		geom->setNormalArray(norm_arr);
		geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
		geod->addDrawable(geom.get());

		osg::StateSet* state = geod->getOrCreateStateSet();

		//osgDB::writeImageFile(*(image.get()), "D:/frame_pose2.png");
		//osg::Image* image2 = osgDB::readImageFile("D:/frame_pose.png");


		//将图像关联到Texture 2D对象
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setDataVariance(osg::Object::DYNAMIC);
		//tex->setImage(image);
		tex->setUpdateCallback(clb);

		// 创建纹理对象后,释放内部的ref_ptr<Image>,删除Image图像。
		tex->setUnRefImageDataAfterApply(false);
		state->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);//关联材质属性与模式到材质单元0
		state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

		state->setMode(GL_BLEND, osg::StateAttribute::ON);
		state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);


		////打开混合
		//osg::BlendFunc* bf = new osg::BlendFunc(
		//	osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		//state->setAttributeAndModes(bf);

		//// 打开Alpha测试
		//osg::AlphaFunc* af = new osg::AlphaFunc(
		//	osg::AlphaFunc::GREATER, 0.05f);
		//state->setAttributeAndModes(af);

		geod->setStateSet(state);
		
		return geod.get();
	}

	void removeArImageUpdateCallback(osg::Geode* geode, ArImageUpdateCallback* clb)
	{
		osg::StateSet* ar_ss = geode->getStateSet();
		osg::StateAttribute* ar_sta = ar_ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE);
		if (ar_sta)
		{
			if (ar_sta->getUpdateCallback() == clb)
			{
				ar_sta->setUpdateCallback(nullptr);
			}
		}
		
	}


	ArImageReceiverThread::ArImageReceiverThread() :OpenThreads::Thread()
	{
		_max_que_size = 2;
		_img_que.resize(0);
	}

	ArImageReceiverThread::~ArImageReceiverThread()
	{
		this->clear();
	}

	int ArImageReceiverThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void ArImageReceiverThread::run()
	{
		_done = false;
		do
		{
			YieldCurrentThread();

		} while (!_done);
	}

	void ArImageReceiverThread::setImage(
		int w, int h,
		GLenum pixelFormat, GLenum type,
		unsigned char* data)
	{
		osg::ref_ptr<osg::Image> _img = new osg::Image;
		_img->allocateImage(w, h, 1, pixelFormat, type, 1);
		_img->setAllocationMode(osg::Image::NO_DELETE);

		int img_s = w;
		int img_t = h;
		int img_r = 1;
		GLenum pix_fmt = pixelFormat;
		GLenum data_type = type;
		unsigned int packing = 1;
		GLint inTexFmt = GL_RGB;
		unsigned int newTotalSize = _img->computeRowWidthInBytes(img_s,
			pixelFormat, type, packing) * img_t * img_r;
		unsigned char* im_data = _img->data();
		memcpy(im_data, data, newTotalSize * sizeof(unsigned char));
		_img->flipVertical();

		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_img_que.size() > _max_que_size)
		{
			_img_que.erase(_img_que.begin(), _img_que.begin() + 1);
		}

		_img_que.push_back(_img);
	}

	bool ArImageReceiverThread::getImage(osg::ref_ptr<osg::Image>& img)
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (isDirty())
		{
			img = _img_que.front();
			_img_que.pop_front();
			return true;
		}
		return false;
	}

	void ArImageReceiverThread::clear()
	{
		OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
		if (_img_que.size() > 0)
		{
			_img_que.clear();
		}
	}

	ArImageUpdateCallback::ArImageUpdateCallback(ArImageReceiverThread* p_thread)
		:osg::StateAttributeCallback(), m_thread(p_thread)
	{
	}

	ArImageUpdateCallback::~ArImageUpdateCallback()
	{

	}

	void ArImageUpdateCallback::operator()(osg::StateAttribute* st, osg::NodeVisitor*)
	{
		if (!m_thread)
		{
			return;
		}

		osg::Texture2D* tex = dynamic_cast<osg::Texture2D*>(st);

		if (!tex)
		{
			return;
		}

		osg::ref_ptr<osg::Image> img = nullptr;
		if (m_thread->getImage(img))
		{
			tex->setImage(img);
			tex->dirtyTextureObject();
		}
	}

}