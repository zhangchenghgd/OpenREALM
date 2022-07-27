#include "TileGridMap.h"
#include <gdal_priv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace MyREALM
{
	TileGridMap::TileGridMap() :TileGridMap(0, 0,
		CV_32FC1, osg::Vec2(0, 0), 1.0)
	{

	}

	TileGridMap::TileGridMap(int _w, int _h, int _data_type,
		const osg::Vec2& _ori, double _gsd,
		double _nodata) :ori(_ori),
		gsd(_gsd), nodata_value(_nodata)
	{
		map = cv::Mat(_w, _h, _data_type);

		if (_data_type == CV_32FC1)
		{
			map.setTo(cv::Scalar(nodata_value));
		}
		else if (_data_type == CV_8UC3)
		{
			map.setTo(cv::Scalar(255, 255, 255));
		}

	}

	TileGridMap::~TileGridMap()
	{
		if (!map.empty())
		{
			map.release();
		}
	}

	WorldRect2 TileGridMap::worldRect() const
	{
		return WorldRect2(ori.x(), ori.y(), map.cols * gsd, map.rows * gsd);
	}

	TileGridMap::Ptr TileGridMap::subMap(int sub_w, int sub_h, 
		osg::Vec2 sub_ori, double sub_gsd, bool only_overlap)
	{
		osg::Vec2 rt = ori + osg::Vec2(map.cols, map.rows) * gsd;

		osg::Vec2 sub_rt = sub_ori + osg::Vec2(sub_w, sub_h) * sub_gsd;

		double min_c_d = (sub_ori.x() - ori.x()) / gsd;
		double min_r_d = map.rows - (sub_rt.y() - ori.y()) / gsd;
		double max_c_d = (sub_rt.x() - ori.x()) / gsd;
		double max_r_d = map.rows - (sub_ori.y() - ori.y()) / gsd;

		int min_c = std::round(min_c_d);
		int min_r = std::round(min_r_d);
		int max_c = std::round(max_c_d);
		int max_r = std::round(max_r_d);

		int sub_w_0 = max_c - min_c;
		int sub_h_0 = max_r - min_r;

		cv::Rect rect_0(0, 0, map.cols, map.rows);
		cv::Rect rect_1(min_c, min_r, sub_w_0, sub_h_0);
		cv::Rect rect_overlap = rect_0 & rect_1;
		cv::Rect rect_overlap2(rect_overlap.x - rect_1.x, rect_overlap.y - rect_1.y,
			rect_overlap.width, rect_overlap.height);

		if (rect_overlap.area() == 0) { return nullptr; }

		TileGridMap::Ptr submap = std::make_shared<TileGridMap>();

		cv::Mat sub_map_0 = cv::Mat(map, rect_overlap);

		if (only_overlap)
		{
			min_c = rect_overlap.x;
			min_r = rect_overlap.y;
			max_c = rect_overlap.x + rect_overlap.width;
			max_r = rect_overlap.y + rect_overlap.height;
			sub_w_0 = rect_overlap.width;
			sub_h_0 = rect_overlap.height;

			double world_h = sub_h_0 * gsd;
			double world_w = sub_w_0 * gsd;

			int sub_w2 = std::round(world_w / sub_gsd);
			int sub_h2 = std::round(world_h / sub_gsd);

			submap->ori.set(ori.x() + min_c * gsd, ori.y() + (map.rows - max_r) * gsd);
			submap->gsd = world_w / sub_w2;

			cv::resize(sub_map_0, submap->map, cv::Size(sub_w2, sub_h2));
		}
		else
		{
			cv::Mat map2;

			//定义平移矩阵
			cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);
			t_mat.at<float>(0, 0) = 1;
			t_mat.at<float>(0, 2) = rect_overlap2.x; //水平平移量
			t_mat.at<float>(1, 1) = 1;
			t_mat.at<float>(1, 2) = rect_overlap2.y; //竖直平移量

			cv::warpAffine(sub_map_0, map2, t_mat, rect_1.size());

			double world_h = sub_h_0 * gsd;
			double world_w = sub_w_0 * gsd;

			int sub_w2 = std::round(world_w / sub_gsd);
			int sub_h2 = std::round(world_h / sub_gsd);

			submap->ori.set(ori.x() + min_c * gsd, ori.y() + (map.rows - max_r) * gsd);
			submap->gsd = world_w / sub_w2;

			cv::resize(map2, submap->map, cv::Size(sub_w2, sub_h2));
		}

		return submap;
	}

	bool TileGridMap::updateMap(TileGridMap::Ptr update_tile, 
		const WorldRect2& update_world_rect)
	{
		double oox = update_world_rect.x;
		double ooy = update_world_rect.y;

		int start_c = (update_world_rect.x - ori.x()) / gsd;
		int start_r = map.rows - ((update_world_rect.y + update_world_rect.height - ori.y()) / gsd);

		int iw = update_world_rect.width / gsd;
		int ih = update_world_rect.height / gsd;

		TileGridMap::Ptr submap = update_tile->subMap(iw, ih, osg::Vec2(oox, ooy), gsd, false);
		if (submap)
		{
			submap->map.copyTo(this->map(
				cv::Rect(start_c, start_r, submap->map.cols, submap->map.rows)));
		}

		return false;
	}

	bool TileGridMap::readFromGDAL(const std::string& filename)
	{
		GDALDataset* img = (GDALDataset*)GDALOpen(filename.c_str(), GA_ReadOnly);
		if (!img)
		{
			return false;
		}

		//读取基本参数
		int imgWidth = img->GetRasterXSize();   //图像宽度
		int imgHeight = img->GetRasterYSize();  //图像高度
		int bandNum = img->GetRasterCount();    //波段数
		int depth = GDALGetDataTypeSize(img->GetRasterBand(1)->GetRasterDataType()) / 8;    //图像深度

		//获取地理坐标信息
		double padfTransform[6];
		if (img->GetGeoTransform(padfTransform) == CE_Failure)
		{
			GDALClose(img);
			return false;
		}
		double startX = padfTransform[0] + 0.5 * padfTransform[1];			//左上角点坐标X
		double dX = padfTransform[1];			//X方向的分辨率		
		double startY = padfTransform[3] + padfTransform[5] * imgHeight - 0.5 * padfTransform[5];			//左下角点坐标Y
		double dY = -padfTransform[5];			//Y方向的分辨率

		nodata_value = img->GetRasterBand(1)->GetNoDataValue();

		//申请buf
		int bufWidth = imgWidth;
		int bufHeight = imgHeight;
		size_t imgBufNum = (size_t)bufWidth * bufHeight * bandNum;
		float* imgBuf = new float[imgBufNum];

		//读取
		size_t imgBufOffset = (size_t)bufWidth * (bufHeight - 1) * bandNum;
		img->RasterIO(GF_Read, 0, 0, bufWidth, bufHeight, imgBuf + imgBufOffset, bufWidth, bufHeight,
			GDT_Float32, bandNum, nullptr, bandNum * depth, -bufWidth * bandNum * depth, depth);

		this->gsd = dX;
		this->ori.set(startX, startY);
		this->map = cv::Mat(bufHeight, bufWidth, CV_32FC1);
		memcpy(this->map.data, imgBuf, imgBufNum * sizeof(float));
		cv::flip(this->map, this->map, 0);


		//释放
		delete[] imgBuf;
		imgBuf = nullptr;

		GDALClose(img);
		return true;
	}

}
