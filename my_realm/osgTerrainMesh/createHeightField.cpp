#include "createHeightField.h"
#include <memory>
#include <iomanip>
#include <gdal_priv.h>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/PagedLOD>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/DelaunayTriangulator>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


osg::Node* createHeightField(const std::string& heightFile, const std::string& texFile)
{
	GDALDataset* img = (GDALDataset*)GDALOpen(heightFile.c_str(), GA_ReadOnly);
	if (!img)
	{
		return nullptr;
	}

	//��ȡ��������
	int imgWidth = img->GetRasterXSize();   //ͼ����
	int imgHeight = img->GetRasterYSize();  //ͼ��߶�
	int bandNum = img->GetRasterCount();    //������
	int depth = GDALGetDataTypeSize(img->GetRasterBand(1)->GetRasterDataType()) / 8;    //ͼ�����

	//��ȡ����������Ϣ
	double padfTransform[6];
	if (img->GetGeoTransform(padfTransform) == CE_Failure)
	{
		return nullptr;
	}
	double startX = padfTransform[0] + 0.5 * padfTransform[1];			//���Ͻǵ�����X
	double dX = padfTransform[1];			//X����ķֱ���		
	double startY = padfTransform[3] + padfTransform[5] * imgHeight - 0.5 * padfTransform[5];			//���½ǵ�����Y
	double dY = -padfTransform[5];			//Y����ķֱ���

	double nodata_value = img->GetRasterBand(1)->GetNoDataValue();

	//����buf
	int bufWidth = imgWidth;
	int bufHeight = imgHeight;
	size_t imgBufNum = (size_t)bufWidth * bufHeight * bandNum;
	float* imgBuf = new float[imgBufNum];

	//��ȡ
	size_t imgBufOffset = (size_t)bufWidth * (bufHeight - 1) * bandNum;
	img->RasterIO(GF_Read, 0, 0, bufWidth, bufHeight, imgBuf + imgBufOffset, bufWidth, bufHeight,
		GDT_Float32, bandNum, nullptr, bandNum * depth, -bufWidth * bandNum * depth, depth);

	//���岢���ø߶��ļ�
	osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField();
	heightField->allocate(imgWidth, imgHeight);			//����ռ�
	heightField->setOrigin(osg::Vec3(startX, startY, 0));			//��ʼλ��	
	heightField->setXInterval(dX);			//���X
	heightField->setYInterval(dY);			//���Y
	heightField->setSkirtHeight(1.0f);


	//���߶�ֵ
	for (int r = 0; r < imgHeight; r++)
	{
		for (int c = 0; c < imgWidth; c++)
		{
			size_t m = (size_t)r * imgWidth + c;
			if (imgBuf[m] != nodata_value)
			{
				heightField->setHeight(c, r, imgBuf[m]);
			}
		}
	}

	//�ͷ�
	delete[] imgBuf;
	imgBuf = nullptr;

	//�ڵ�
	osg::Geode* geode = new osg::Geode();
	osg::ref_ptr<osg::ShapeDrawable> heightShape = new osg::ShapeDrawable(heightField.get());
	geode->addDrawable(heightShape);

	//��������
	osg::ref_ptr<osg::Image> texImage = osgDB::readImageFile(texFile);
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	tex->setImage(texImage);
	tex->setDataVariance(osg::Object::DYNAMIC);

	//��Ⱦ״̬
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	geode->setStateSet(stateset.get());

	return geode;
}



osg::Node* createHeightField2(const std::string& heightFile, const std::string& texFile)
{
	GDALDataset* img = (GDALDataset*)GDALOpen(heightFile.c_str(), GA_ReadOnly);
	if (!img)
	{
		return nullptr;
	}

	//��ȡ��������
	int imgWidth = img->GetRasterXSize();   //ͼ����
	int imgHeight = img->GetRasterYSize();  //ͼ��߶�
	int bandNum = img->GetRasterCount();    //������
	int depth = GDALGetDataTypeSize(img->GetRasterBand(1)->GetRasterDataType()) / 8;    //ͼ�����

	//��ȡ����������Ϣ
	double padfTransform[6];
	if (img->GetGeoTransform(padfTransform) == CE_Failure)
	{
		return nullptr;
	}
	double startX = padfTransform[0] + 0.5 * padfTransform[1];			//���Ͻǵ�����X
	double dX = padfTransform[1];			//X����ķֱ���		
	double startY = padfTransform[3] + padfTransform[5] * imgHeight - 0.5 * padfTransform[5];			//���½ǵ�����Y
	double dY = -padfTransform[5];			//Y����ķֱ���

	double nodata_value = img->GetRasterBand(1)->GetNoDataValue();

	//����buf
	int bufWidth = imgWidth;
	int bufHeight = imgHeight;
	size_t imgBufNum = (size_t)bufWidth * bufHeight * bandNum;
	float* imgBuf = new float[imgBufNum];

	//��ȡ
	size_t imgBufOffset = (size_t)bufWidth * (bufHeight - 1) * bandNum;
	img->RasterIO(GF_Read, 0, 0, bufWidth, bufHeight, imgBuf + imgBufOffset, bufWidth, bufHeight,
		GDT_Float32, bandNum, nullptr, bandNum * depth, -bufWidth * bandNum * depth, depth);


	osg::ref_ptr<osg::Vec3Array> verts_arr = new osg::Vec3Array(imgWidth * imgHeight);
	osg::ref_ptr<osg::Vec3Array> norm_arr = new osg::Vec3Array(imgWidth * imgHeight);
	osg::ref_ptr<osg::Vec2Array> texture_arr = new osg::Vec2Array(imgWidth * imgHeight);
	osg::UIntArray* indxs = new  osg::UIntArray((imgWidth - 1) * (imgHeight - 1) * 2 * 3);

	//���߶�ֵ
	for (size_t r = 0; r < imgHeight; r++)
	{
		for (size_t c = 0; c < imgWidth; c++)
		{
			size_t m = (size_t)r * imgWidth + c;
			double vx = 0 + c * dX;
			double vy = 0 + r * dY;
			if (imgBuf[m] != nodata_value)
			{
				// heightField->setHeight(c, r, imgBuf[m]);

				double vz = imgBuf[m];
				verts_arr->at(m).set(vx, vy, vz);
				norm_arr->at(m).set(osg::Vec3(vx, vy, vz) - osg::Vec3(100, 100, 0));
			}
			else
			{
				verts_arr->at(m).set(vx, vy, 0);
				norm_arr->at(m).set(0.0, 0.0, 1.0);
			}

			texture_arr->at(m).set(1.0 * c / imgWidth, 1.0 * r / imgHeight);

			if (r < imgHeight - 1 && c < imgWidth - 1)
			{
				size_t v0 = r * imgWidth + c;
				size_t v1 = (r + 1) * imgWidth + c;
				size_t v2 = (r + 1) * imgWidth + c + 1;
				size_t v3 = r * imgWidth + c + 1;

				size_t f1_0 = ((imgWidth - 1) * r + c) * 2 * 3;
				size_t f1_1 = f1_0 + 1;
				size_t f1_2 = f1_0 + 2;
				size_t f2_0 = f1_0 + 3;
				size_t f2_1 = f1_0 + 4;
				size_t f2_2 = f1_0 + 5;
				/*faces[f1_0] = v0;
				faces[f1_1] = v1;
				faces[f1_2] = v2;
				faces[f2_0] = v0;
				faces[f2_1] = v2;
				faces[f2_2] = v3;*/

				/*indxs->at(f1_0) = v0;
				indxs->at(f1_1) = v1;
				indxs->at(f1_2) = v2;
				indxs->at(f2_0) = v0;
				indxs->at(f2_1) = v2;
				indxs->at(f2_2) = v3;*/

				indxs->at(f1_0) = v0;
				indxs->at(f1_1) = v2;
				indxs->at(f1_2) = v1;
				indxs->at(f2_0) = v0;
				indxs->at(f2_1) = v3;
				indxs->at(f2_2) = v2;
			}
		}
	}


	//�ͷ�
	delete[] imgBuf;
	imgBuf = nullptr;

	GDALClose(img);

	//�ڵ�
	osg::Geode* geode = new osg::Geode();
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts_arr);
	geom->setTexCoordArray(0, texture_arr);

	osg::ref_ptr<osg::DrawElementsUInt> triangles =
		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, indxs->begin(), indxs->end());

	geom->addPrimitiveSet(triangles.get());

	geom->setUseVertexBufferObjects(true);
	geom->setUseDisplayList(false);
	geom->setDataVariance(osg::Object::DYNAMIC);

	osgUtil::SmoothingVisitor::smooth(*geom.get());

	geode->addDrawable(geom);


	//��������
	osg::ref_ptr<osg::Image> texImage = osgDB::readImageFile(texFile); //
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	tex->setImage(texImage);
	tex->setDataVariance(osg::Object::DYNAMIC);

	//��Ⱦ״̬
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.986, 0.986, 0.811, 1.0));
	material->setAmbient(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	material->setShininess(osg::Material::FRONT, 90);
	material->setColorMode(osg::Material::AMBIENT);
	stateset->setAttribute(material.get());

	geode->setStateSet(stateset.get());

	return geode;
}


