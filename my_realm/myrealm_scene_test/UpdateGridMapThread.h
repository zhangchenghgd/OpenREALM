#ifndef MYREALM_UPDATEGRIDMAPTHREAD_H
#define MYREALM_UPDATEGRIDMAPTHREAD_H

#include <cassert>
#include <memory>
#include <map>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>
#include "TileGridMap.h"
#include "TileMesh.h"

namespace MyREALM
{

	class TileMeshUpdateThread;

	class UpdateGridMapThread : public OpenThreads::Thread
	{
	public:
		UpdateGridMapThread(
			const std::string& p_dsm_path,
			const std::string& p_dom_path,
			MyREALM::TileMesh* p_tile_mesh);

		virtual int cancel();
		virtual void run();

		void updateGridMap();

	protected:
		OpenThreads::Mutex _mutex;
		MyREALM::TileMesh* _tile_mesh;

		TileGridMap::Ptr _dsm;
		TileGridMap::Ptr _dom;

		cv::Vec2d _start_vec;
		cv::Vec2d _end_vec;
		int _max_iter_num;
		int _move_step;
		int _move_block_size;


		int _update_it;
		bool _in_updating;

		bool _done;
		bool _dirty;
	};

}

#endif  // !MYREALM_UPDATEGRIDMAPTHREAD_H