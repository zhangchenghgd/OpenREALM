#ifndef MYREALM_SCENE_TILEGRIDMAP_H
#define MYREALM_SCENE_TILEGRIDMAP_H

#include "myrealm_scene_exports.h"
#include "WorldRect2.h"
#include <cassert>
#include <memory>
#include <map>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include <opencv2/core.hpp>

namespace MyREALM
{
	struct MyREALM_Scene_API TileGridMap
	{
	public:
		using Ptr = std::shared_ptr<TileGridMap>;
		using ConstPtr = std::shared_ptr<const TileGridMap>;

		cv::Mat map;
		osg::Vec2 ori;  // 左下坐标原点
		double gsd;     // 地面分辨率
		double nodata_value;

		TileGridMap();

		TileGridMap(int _w, int _h, int _data_type,
			const osg::Vec2& _ori, double _gsd,
			double _nodata = -9999.0);

		~TileGridMap();

		WorldRect2 worldRect() const;

		TileGridMap::Ptr subMap(int sub_w, int sub_h, osg::Vec2 sub_ori,
			double sub_gsd, bool only_overlap = true);

		bool updateMap(TileGridMap::Ptr update_tile, const WorldRect2& update_wordl_rect);

		bool readFromGDAL(const std::string& filename);

	};
}

#endif // !MYREALM_SCENE_TILEGRIDMAP_H