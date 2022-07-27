#include "TileMeshUpdatable.h"
#include "TileMeshLOD.h"
#include <osg/PagedLOD>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReadFile>
#include <osgDB/DatabasePager>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace MyREALM
{
	TileMeshUpdatable::TileMeshUpdatable(
		const std::string& _prefix,
		const osg::Vec2& _ori, double _tile_size,
		double _highest_gsd, int _level_num,
		const std::string& _dirname)
		:m_dirname(_dirname),m_prefix(_prefix)
	{
		m_updated = false;
		double new_tile_size = _tile_size * 1.02;

		double root_gsd = _highest_gsd * pow(2, _level_num);
		m_tile_tree = std::make_shared<TileLodTree>(
			_prefix, 0, new_tile_size, _ori, root_gsd);
		m_tile_tree->initChildren(_level_num);

		int grid_size = std::ceil(new_tile_size / _highest_gsd);
		double grid_gsd = (new_tile_size) / grid_size;

		m_dsm = std::make_shared<TileGridMap>(
			grid_size, grid_size, CV_32FC1, _ori, grid_gsd);

		m_dom = std::make_shared<TileGridMap>(
			grid_size, grid_size, CV_8UC3, _ori, grid_gsd);

	}


	TileMeshUpdatable::~TileMeshUpdatable()
	{

	}

	
	bool TileMeshUpdatable::updateTileGridMap(
		TileGridMap::Ptr update_dsm_map,
		TileGridMap::Ptr update_dom_map,
		const WorldRect2& update_world_rect)
	{

		WorldRect2 update_rect = update_world_rect.empty()
			? update_dsm_map->worldRect() : update_world_rect;

		WorldRect2 dsm_rect = m_dsm->worldRect();
		WorldRect2 overlap = update_rect & dsm_rect;
		if (overlap.empty()) { return false; }

		//m_mutex.lock();
		m_dsm->updateMap(update_dsm_map, overlap);
		m_dom->updateMap(update_dom_map, overlap);

		bool update_suc = updateTilePageLod(m_tile_tree.get(), 
			m_dsm, m_dom, m_dirname, overlap);
		if (update_suc)
		{
			m_updated = true;
		}


		//m_mutex.unlock();

		return update_suc;
	}

	bool TileMeshUpdatable::getUpdated()
	{
		bool ret_update = m_updated;
		//m_mutex.lock();
		if (m_updated)
		{
			std::cout << "Tile mesh file updated: " << m_tile_tree->getPrefix() << std::endl;
			m_updated = false;
		}
		//m_mutex.unlock();

		return ret_update;
	}

	std::string TileMeshUpdatable::getTileMeshFilename() const
	{
		return m_tile_tree->getPageLODFilename(m_dirname);
	}

	TileLodTree* TileMeshUpdatable::getTileTree()
	{
		return m_tile_tree.get();
	}

	TileMeshUpdateCallback::TileMeshUpdateCallback(
		TileMeshUpdatable* p_updatable)
		:m_updatable(p_updatable)
	{
		m_timer = new osg::Timer;
	}

	TileMeshUpdateCallback::~TileMeshUpdateCallback()
	{

	}

	void TileMeshUpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::Group* grp = node->asGroup();
		if (!grp)
		{
			return;
		}

		if (grp->getNumChildren() > 1 && m_timer->time_m() >= 2000)
		{
			grp->removeChildren(0, grp->getNumChildren() - 1);
		}

		if (m_updatable->getUpdated())
		{
			std::string root_node_filename = m_updatable->getTileMeshFilename();

			if (osgDB::fileExists(root_node_filename))
			{
				osg::ref_ptr<osg::Node> node =
					osgDB::readNodeFile(root_node_filename);
				grp->addChild(node);

				std::cout << "Tile mesh node loaded: "
					<< m_updatable->getTileTree()->getPrefix() << std::endl;
				m_timer->setStartTick();
			}

		}
	}

}

