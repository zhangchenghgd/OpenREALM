#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>

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

		return geod;
	}
}