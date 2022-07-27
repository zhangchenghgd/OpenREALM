#include "UpdateGridMapThread.h"
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer> //优化器
#include <osgUtil/Simplifier> //简化几何体
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "TileMeshUpdatable.h"


namespace MyREALM
{
	UpdateGridMapThread::UpdateGridMapThread(
		const std::string& p_dsm_path,
		const std::string& p_dom_path,
		MyREALM::TileMesh* p_tile_mesh)
		: OpenThreads::Thread(), _tile_mesh(p_tile_mesh)
	{
		_dsm = std::make_shared<TileGridMap>();
		_dsm->readFromGDAL(p_dsm_path);
		_dsm->gsd = 0.8;
		_dsm->ori.set(0., 0.);

		_dom = std::make_shared<TileGridMap>();
		_dom->map = cv::imread(p_dom_path);
		_dom->gsd = 0.8;
		_dom->ori.set(0., 0.);
	}

	int UpdateGridMapThread::cancel()
	{
		_done = true;
		while (isRunning()) YieldCurrentThread();
		return 0;
	}

	void UpdateGridMapThread::run()
	{
		
		_in_updating = false;
		_done = false;
		_dirty = true;

		_update_it = 0;
		double start_c = 1200;
		double start_r = 3050;
		double end_c = 3400;
		double end_r = 1080;
		_move_block_size = 520;
		_move_step = 50;

		_start_vec = cv::Vec2d(start_c, start_r);
		_end_vec = cv::Vec2d(end_c, end_r);
		cv::Vec2d fvec = (_end_vec - _start_vec);
		_max_iter_num = cv::norm(fvec) / _move_step;
		
		do
		{
			if (_in_updating)
			{
				YieldCurrentThread();
			}
			else
			{
				this->updateGridMap();
				this->microSleep(2000000);
			}

		} while (!_done);
	}

	void UpdateGridMapThread::updateGridMap()
	{
		_in_updating = true;
		
		cv::Vec2d fvec = (_end_vec - _start_vec);
		fvec = cv::normalize(fvec);

		cv::Vec2d cur = _start_vec + fvec * _move_step * _update_it;

		double mox = cur[0] - _move_block_size * 0.5;
		double moy = (_dsm->map.rows - cur[1]) - _move_block_size * 0.5;

		double ox = _dsm->ori.x() + mox * _dsm->gsd;
		double oy = _dsm->ori.y() + moy * _dsm->gsd;

		osg::Vec2 ori(ox, oy);

		TileGridMap::Ptr sub_dsm = _dsm->subMap(_move_block_size, _move_block_size, ori, _dsm->gsd, false);
		TileGridMap::Ptr sub_dom = _dom->subMap(_move_block_size, _move_block_size, ori, _dom->gsd, false);
		
		_tile_mesh->updateTileGridMap(sub_dsm, sub_dom, sub_dom->worldRect());
		
		_update_it++;
		_in_updating = false;

		if (_update_it >= _max_iter_num)
		{
			_done = true;
		}
	}

}