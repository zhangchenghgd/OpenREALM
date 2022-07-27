#ifndef MYREALM_SCENE_TILEMESHLOD_H
#define MYREALM_SCENE_TILEMESHLOD_H

#include "myrealm_scene_exports.h"
#include "TileGridMap.h"
#include "TileLodTree.h"
#include <osg/LOD>

namespace MyREALM
{
	bool updateTilePageLod(TileLodTree* lod_tree, TileGridMap::Ptr dsm,
		TileGridMap::Ptr dom, const std::string& dirname, 
		const WorldRect2& update_world_rect = WorldRect2());

	void removeTilePageLodFiles(TileLodTree* lod_tree,
		const std::string& dirname, int& rm_file_count);

}

#endif // !MYREALM_SCENE_TILEMESHLOD_H