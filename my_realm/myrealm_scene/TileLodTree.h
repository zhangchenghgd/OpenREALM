#ifndef MYREALM_SCENE_TILELODTREE_H
#define MYREALM_SCENE_TILELODTREE_H

#include "myrealm_scene_exports.h"
#include "WorldRect2.h"
#include <memory>
#include <string>
#include <osg/Vec2>
#include <osg/LOD>
#include <opencv2/core.hpp>

namespace MyREALM
{
	/**
	 * @brief TileLod树结构.
	 */
	class MyREALM_Scene_API TileLodTree
	{
	public:
		using Ptr = std::shared_ptr<TileLodTree>;
		using ConstPtr = std::shared_ptr<const TileLodTree>;

		TileLodTree();

		TileLodTree(const std::string& _prefix, int _level, int _tile_size,
			const osg::Vec2& _ori, double _gsd);

		~TileLodTree();

		std::string getPrefix() const;

		std::string getName() const;

		std::string getPageLODFilename(const std::string& dirname = "") const;

		std::string getGeodeFilename(const std::string& dirname = "") const;

		void initChildren(int max_level = 0);

		int level() const { return m_level; }

		void setLevel(int _level) { m_level = _level; }

		double tileSize() const { return m_tile_size; }

		void setTileSize(double _size) { m_tile_size = _size; }

		TileLodTree* parent() { return m_parent; }

		int childrenNum() const { return m_children_num; }

		TileLodTree* child(int index)
		{
			if (index < m_children_num && index >= 0)
			{
				return &m_children[index];
			}
			else
			{
				return nullptr;
			}
		}

		int childIndex() { return m_child_index; }

		const osg::Vec2& ori() const { return m_ori; }

		void setOri(const osg::Vec2& o_ri) { m_ori = m_ori; }

		double gsd() const { return m_gsd; }

		void setGSD(double _gsd) { m_gsd = _gsd; }

		WorldRect2 worldRect() const;

		osg::LOD* lod() { return m_lod; }

		void setLOD(osg::LOD* lod) { m_lod = lod; }

	private:
		std::string m_prefix;    // 前缀，例如：Tile_0001_0001
		int m_level;             // 层级
		double m_tile_size;      // Tile世界坐标系大小
		TileLodTree* m_parent;   // 父Tile
		int m_child_index;       // 该节点在父Tile中的索引
		TileLodTree* m_children; // 子Tile
		int m_children_num;      // 子Tile数量
		osg::Vec2 m_ori;         // Tile 世界坐标系原点
		double m_gsd;            // 分辨率
		osg::LOD* m_lod;
	};

	
	MyREALM_Scene_API void printTileLodTree(TileLodTree* tree);

}

#endif // !MYREALM_SCENE_TILELODTREE_H