#ifndef MYREALM_SCENE_TILEMESH_H
#define MYREALM_SCENE_TILEMESH_H

#include "myrealm_scene_exports.h"
#include "TileGridMap.h"
#include "TileLodTree.h"
#include "TileMeshUpdatable.h"
#include "TileThreadPool.h"
#include <map>
#include <osg/Node>
#include <osg/Group>

namespace MyREALM
{
	class MyREALM_Scene_API TileMesh
	{
	public:
		using Ptr = std::shared_ptr<TileMesh>;
		using ConstPtr = std::shared_ptr<const TileMesh>;

		using TileIndex = std::pair<int, int>;

		TileMesh(const osg::Vec2& _ori, float _tile_size, 
			float _highest_gsd, int _level,
			const std::string& _root_dirname, 
			size_t _max_threads_num = 4);

		~TileMesh();

		void updateTileGridMap(
			TileGridMap::Ptr update_dsm_map,
			TileGridMap::Ptr update_dom_map,
			const WorldRect2& update_world_rect = WorldRect2());

		osg::ref_ptr<osg::Group> meshNode();

	private:
		osg::Vec2 m_ori;
		float m_tile_size;
		float m_highest_gsd;
		int m_level;
		std::string m_root_dirname;
		osg::ref_ptr<osg::Group> m_meshNodeGroup;
		std::map<TileIndex, TileMeshUpdatable::Ptr> m_tileUpdatabels;
		TileThreadPool::Ptr m_tileThreadPool;
	};
}

#endif // !MYREALM_SCENE_TILEMESH_H