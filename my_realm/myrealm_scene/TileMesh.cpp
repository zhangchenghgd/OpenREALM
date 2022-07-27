#include "TileMesh.h"
#include <sstream>
#include <iomanip>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <realm_io/utilities.h>

namespace MyREALM
{
	TileMesh::TileMesh(const osg::Vec2& _ori,
		float _tile_size, float _highest_gsd,
		int _level, const std::string& _root_dirname, 
		size_t _max_threads_num) :
		m_ori(_ori), m_tile_size(_tile_size),
		m_highest_gsd(_highest_gsd), m_level(_level),
		m_root_dirname(_root_dirname)
	{
		m_meshNodeGroup = new osg::Group;

		m_tileThreadPool = std::make_shared<TileThreadPool>(_max_threads_num);

	}

	TileMesh::~TileMesh()
	{
		for (int i = 0; i < m_meshNodeGroup->getNumChildren(); ++i)
		{
			osg::Node* tile_group = m_meshNodeGroup->getChild(i);
			osg::Callback* clb = tile_group->getUpdateCallback();
			if (clb)
			{
				tile_group->removeUpdateCallback(clb);
			}
		}

		m_tileUpdatabels.clear();
	}

	void TileMesh::updateTileGridMap(TileGridMap::Ptr update_dsm_map,
		TileGridMap::Ptr update_dom_map, const WorldRect2& update_world_rect)
	{
		WorldRect2 update_rect = update_world_rect.empty() ?
			update_dsm_map->worldRect() : update_world_rect;

		int col_min = floor((update_rect.bottomLeft().x - m_ori.x()) / m_tile_size);
		int row_min = floor((update_rect.bottomLeft().y - m_ori.y()) / m_tile_size);
		int col_max = floor((update_rect.topRight().x - m_ori.x()) / m_tile_size);
		int row_max = floor((update_rect.topRight().y - m_ori.y()) / m_tile_size);

		for (int c = col_min; c < col_max + 1; c++)
		{
			for (int r = row_min; r < row_max + 1; r++)
			{
				TileIndex tile_index(c, r);
				TileMeshUpdatable::Ptr updateable;
				std::map<TileIndex, TileMeshUpdatable::Ptr>::iterator
					td_it = m_tileUpdatabels.find(tile_index);
				if (td_it != m_tileUpdatabels.end())
				{
					updateable = td_it->second;
				}
				else
				{
					osg::Vec2 tile_ori(
						m_ori.x() + c * m_tile_size,
						m_ori.y() + r * m_tile_size);

					std::string col_prefix_chr = c >= 0 ? "+" : "-";
					std::string row_prefix_chr = r >= 0 ? "+" : "-";

					std::stringstream ss;
					ss << "Tile_" <<
						col_prefix_chr << std::setfill('0') << std::setw(4) << abs(c)
						<< "_" <<
						row_prefix_chr << std::setfill('0') << std::setw(4) << abs(r);
					std::string tile_name = ss.str();
					std::string tile_dirname = osgDB::concatPaths(m_root_dirname, tile_name);
					if (!realm::io::dirExists(tile_dirname))
					{
						realm::io::createDir(tile_dirname);
					}
					updateable = std::make_shared<TileMeshUpdatable>(tile_name,
						tile_ori, m_tile_size, m_highest_gsd, m_level, tile_dirname);
					osg::ref_ptr<MyREALM::TileMeshUpdateCallback> tileCLB =
						new MyREALM::TileMeshUpdateCallback(updateable.get());
					osg::ref_ptr<osg::Group> tile_group = new osg::Group;
					tile_group->setName(tile_name);
					tile_group->setUpdateCallback(tileCLB);
					m_tileUpdatabels[tile_index] = updateable;
					m_meshNodeGroup->addChild(tile_group);
				}

				auto meshTileUpdateFunc = std::bind(
					&TileMeshUpdatable::updateTileGridMap, updateable.get(),
					std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

				auto result = m_tileThreadPool->enqueue(
					meshTileUpdateFunc, update_dsm_map, update_dom_map, update_world_rect);

			}
		}
	}

	osg::ref_ptr<osg::Group> TileMesh::meshNode()
	{
		return m_meshNodeGroup;
	}

}
