#ifndef MYREALM_SCENE_TILEMESHUPDATABLE_H
#define MYREALM_SCENE_TILEMESHUPDATABLE_H

#include "myrealm_scene_exports.h"
#include "TileLodTree.h"
#include "TileGridMap.h"
#include <mutex>
#include <osg/NodeCallback>
#include <osg/Timer>
#include <OpenThreads/Thread>

namespace MyREALM
{
	class MyREALM_Scene_API TileMeshUpdatable
	{
	public:
		using Ptr = std::shared_ptr<TileMeshUpdatable>;
		using ConstPtr = std::shared_ptr<const TileMeshUpdatable>;

		TileMeshUpdatable(const std::string& _prefix, 
			const osg::Vec2& _ori, double _tile_size, 
			double _highest_gsd, int _level_num, 
			const std::string& _dirname);

		~TileMeshUpdatable();

		bool updateTileGridMap(
			TileGridMap::Ptr update_dsm_map,
			TileGridMap::Ptr update_dom_map,
			const WorldRect2& update_world_rect = WorldRect2());

		bool getUpdated();

		std::string getTileMeshFilename() const;

		TileLodTree* getTileTree();

	protected:
		std::mutex m_mutex;
		TileLodTree::Ptr m_tile_tree;
		std::string m_prefix;
		std::string m_dirname;
		TileGridMap::Ptr m_dsm;
		TileGridMap::Ptr m_dom;
		bool m_updated;
	};

	class TileMeshUpdateCallback :public osg::NodeCallback
	{
	public:
		TileMeshUpdateCallback(TileMeshUpdatable* p_updatable);
		~TileMeshUpdateCallback();

		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override;

	protected:
		TileMeshUpdatable* m_updatable;
		osg::Timer* m_timer;
	};
}

#endif // !MYREALM_SCENE_TILEMESHUPDATABLE_H