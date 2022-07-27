#include "TileMeshLOD.h"
#include <osg/PagedLOD>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgUtil/SmoothingVisitor>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <realm_io/utilities.h>

namespace MyREALM
{
	
	bool updateTilePageLod(TileLodTree* lod_tree, TileGridMap::Ptr dsm,
		TileGridMap::Ptr dom, const std::string& dirname, 
		const WorldRect2& update_world_rect)
	{
		std::string node_filepath = lod_tree->getPageLODFilename(dirname);

		WorldRect2 update_rect = update_world_rect.empty()
			? lod_tree->worldRect() : update_world_rect;
		WorldRect2 overlap = update_rect & lod_tree->worldRect();
		if (overlap.empty())
		{
			// 如果已经存在该文件，说明lod文件已经生成，无需更新，节点已经创建成功
			// 如果文件不存在，说明该节点lod生成失败
			return osgDB::fileExists(node_filepath);
		}

		double ext_val = lod_tree->tileSize() * 0.01;
		osg::Vec2 sub_ori = lod_tree->ori() - osg::Vec2(ext_val, ext_val);
		double sub_tile_size = lod_tree->tileSize() + 2 * ext_val;
		int imgWidth = std::round(sub_tile_size / lod_tree->gsd());
		int imgHeight = std::round(sub_tile_size / lod_tree->gsd());


		TileGridMap::Ptr sub_dsm = dsm->subMap(imgWidth, imgHeight, sub_ori, lod_tree->gsd(), true);
		if (!sub_dsm) 
		{
			int rm_file_num = 0;
			removeTilePageLodFiles(lod_tree, dirname, rm_file_num);

			return false;
		}
		imgWidth = sub_dsm->map.cols;
		imgHeight = sub_dsm->map.rows;
		double nodata_value = dsm->nodata_value;

		osg::ref_ptr<osg::Vec3Array> verts_arr = new osg::Vec3Array();
		verts_arr->reserve(static_cast<size_t>(imgWidth * imgHeight));
		osg::ref_ptr<osg::Vec2Array> texture_arr = new osg::Vec2Array();
		texture_arr->reserve(static_cast<size_t>(imgWidth * imgHeight));
		osg::ref_ptr<osg::UIntArray> indxs = new  osg::UIntArray();
		indxs->reserve((imgWidth - 1) * (imgHeight - 1) * 2 * 3);

		int* verts_indexs = new int[imgWidth * imgHeight];
		memset(&verts_indexs[0], -1, imgWidth * imgHeight * sizeof(int));

		double sum_z = 0;
		size_t z_num = 0;
		//填充高度值
		for (size_t r = 0; r < imgHeight; r++)
		{
			for (size_t c = 0; c < imgWidth; c++)
			{
				size_t m = (size_t)r * imgWidth + c;
				double vx = sub_dsm->ori.x() + c * sub_dsm->gsd;
				double vy = sub_dsm->ori.y() + r * sub_dsm->gsd;
				float vz = sub_dsm->map.at<float>(sub_dsm->map.rows - r - 1, c);
				if (vz!=NAN && vz != nodata_value && vz > -100)
				{
					verts_arr->push_back(osg::Vec3(vx, vy, vz));
					texture_arr->push_back((osg::Vec2(1.0 * c / imgWidth, 1.0 * r / imgHeight)));
					verts_indexs[m] = z_num;
					sum_z += vz;
					z_num++;
				}
			}
		}

		for (size_t r = 0; r < imgHeight; r++)
		{
			for (size_t c = 0; c < imgWidth; c++)
			{
				//size_t m = (size_t)r * imgWidth + c;

				if (r < imgHeight - 1 && c < imgWidth - 1)
				{
					size_t v0 = r * imgWidth + c;
					size_t v1 = (r + 1) * imgWidth + c;
					size_t v2 = (r + 1) * imgWidth + c + 1;
					size_t v3 = r * imgWidth + c + 1;

					int v0_idx = verts_indexs[v0];
					int v1_idx = verts_indexs[v1];
					int v2_idx = verts_indexs[v2];
					int v3_idx = verts_indexs[v3];
					
					if (v0_idx>=0 && v2_idx>=0 && v1_idx>=0)
					{
						indxs->push_back(v0_idx);
						indxs->push_back(v2_idx);
						indxs->push_back(v1_idx);
					}

					if (v0_idx >= 0 && v3_idx >= 0 && v2_idx >= 0)
					{
						indxs->push_back(v0_idx);
						indxs->push_back(v3_idx);
						indxs->push_back(v2_idx);
					}

				}
			}
		}

		delete[] verts_indexs;

		if (z_num < 4)
		{
			int rm_file_num = 0;
			removeTilePageLodFiles(lod_tree, dirname, rm_file_num);
			return false;
		}

		osg::Vec3 tile_center(
			sub_dsm->ori.x() + sub_dsm->map.cols * sub_dsm->gsd * 0.5,
			sub_dsm->ori.y() + sub_dsm->map.rows * sub_dsm->gsd * 0.5,
			sum_z / z_num);

		//节点
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setVertexArray(verts_arr);
		geom->setTexCoordArray(0, texture_arr);

		osg::ref_ptr<osg::DrawElementsUInt> triangles =
			new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, indxs->begin(), indxs->end());

		geom->addPrimitiveSet(triangles.get());

		geom->setUseVertexBufferObjects(true);
		geom->setUseDisplayList(false);
		geom->setDataVariance(osg::Object::STATIC);

		// 自动生成顶点法向量
		//osgUtil::SmoothingVisitor::smooth(*geom.get());

		geode->addDrawable(geom);

		osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();
		// 纹理相对于地形分辨率提升倍数
		int texture_gsd_scale = 5;
		//设置纹理
		TileGridMap::Ptr sub_dom = dom->subMap(
			imgWidth * texture_gsd_scale, imgHeight * texture_gsd_scale,
			sub_dsm->ori, sub_dsm->gsd / texture_gsd_scale, false);
		if (sub_dom)
		{
			cv::Mat map_rgb;
			cv::cvtColor(sub_dom->map, map_rgb, cv::COLOR_BGR2RGB);
			osg::ref_ptr<osg::Image> _img = new osg::Image;
			_img->allocateImage(map_rgb.cols, map_rgb.rows, 1, GL_RGB, GL_UNSIGNED_BYTE, 1);
			_img->setAllocationMode(osg::Image::USE_NEW_DELETE);

			int img_s = map_rgb.cols;
			int img_t = map_rgb.rows;
			int img_r = 1;
			GLenum pix_fmt = GL_RGB;
			GLenum data_type = GL_UNSIGNED_BYTE;
			unsigned int packing = 1;
			GLint inTexFmt = GL_RGB;
			unsigned int newTotalSize = _img->computeRowWidthInBytes(img_s,
				GL_RGB, GL_UNSIGNED_BYTE, packing) * img_t * img_r;
			unsigned char* im_data = _img->data();
			memcpy(im_data, map_rgb.data, newTotalSize * sizeof(unsigned char));
			_img->flipVertical();

			osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
			tex->setImage(_img);
			tex->setDataVariance(osg::Object::DYNAMIC);

			//渲染状态
			stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

		}

		//stateset->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		geode->setStateSet(stateset.get());

		double range_val = 10000.0 / lod_tree->gsd();

		std::string geod_filename = lod_tree->getGeodeFilename();
		std::string geod_filepath = lod_tree->getGeodeFilename(dirname);
		osgDB::writeNodeFile(*geode.get(), geod_filepath);

		osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD;
		lod->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);

		lod->setFileName(0, geod_filename);
		lod->setRange(0, 0, range_val);

		int child_idx = 1;
		for (int i = 0; i < lod_tree->childrenNum(); ++i)
		{
			TileLodTree* child_tree = lod_tree->child(i);

			std::string child_filename = child_tree->getPageLODFilename();
			if (updateTilePageLod(child_tree, dsm, dom, dirname, update_rect))
			{
				lod->setFileName(child_idx, child_filename);
				lod->setRange(child_idx, range_val, FLT_MAX);
				child_idx++;
			}
		}

		osgDB::writeNodeFile(*lod.get(), node_filepath);
		return true;
	}

	void removeTilePageLodFiles(TileLodTree* lod_tree, 
		const std::string& dirname,
		int& rm_file_count)
	{
		std::string pagelod_filename = lod_tree->getPageLODFilename(dirname);
		std::string geode_filename = lod_tree->getGeodeFilename(dirname);

		for (int i = 0; i < lod_tree->childrenNum(); i++)
		{
			TileLodTree* child_tree = lod_tree->child(i);
			removeTilePageLodFiles(child_tree, dirname, rm_file_count);
		}

		if (realm::io::fileExists(geode_filename))
		{
			if (realm::io::removeFileOrDirectory(geode_filename))
			{
				rm_file_count++;
			}
		}

		if (realm::io::fileExists(pagelod_filename))
		{
			if (realm::io::removeFileOrDirectory(pagelod_filename))
			{
				rm_file_count++;
			}
		}

	}

}
