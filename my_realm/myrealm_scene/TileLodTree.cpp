#include "TileLodTree.h"
#include <iostream>
#include <osgDB/FileNameUtils>


namespace MyREALM
{
	TileLodTree::TileLodTree() :
		m_prefix("Tile_"), m_level(0), m_tile_size(0), m_parent(nullptr),
		m_child_index(-1), m_children_num(0), m_children(nullptr),
		m_ori(0, 0), m_gsd(1.0), m_lod(nullptr)
	{

	}

	TileLodTree::TileLodTree(const std::string& _prefix, int _level, 
		int _tile_size, const osg::Vec2& _ori, double _gsd) :
		m_prefix(_prefix), m_level(_level), m_tile_size(_tile_size),
		m_parent(nullptr), m_child_index(-1), m_children(nullptr),
		m_children_num(0), m_ori(_ori), m_gsd(_gsd), m_lod(nullptr)
	{

	}

	TileLodTree::~TileLodTree()
	{
		if (m_children_num > 0)
		{
			delete[] m_children;
			m_children = nullptr;
		}
	}

	std::string TileLodTree::getPrefix() const
	{
		return m_prefix;
	}

	std::string TileLodTree::getName() const
	{
		/*std::string name = "_" + std::to_string(m_level)
			+ "_" + std::to_string(m_child_index);
		const TileLodTree* parent_tile = this->m_parent;
		while (parent_tile != nullptr)
		{
			name = "_" + std::to_string(parent_tile->m_level)
				+ name;
			parent_tile = parent_tile->m_parent;
		}
		name = m_prefix + name;*/

		std::string name;
		if (m_parent)
		{
			name = m_parent->getName() + "_"
				+ std::to_string(m_child_index);
		}
		else
		{
			name = m_prefix;
		}

		return name;
	}

	std::string TileLodTree::getPageLODFilename(const std::string& dirname) const
	{
		std::string filename = this->getName() + ".osgb";
		std::string filepath = osgDB::concatPaths(dirname, filename);

		return filepath;
	}



	std::string TileLodTree::getGeodeFilename(const std::string& dirname) const
	{
		std::string filename = this->getName() + "_gd.osgb";
		std::string filepath = osgDB::concatPaths(dirname, filename);

		return filepath;
	}

	void TileLodTree::initChildren(int max_level)
	{
		if (m_children_num > 0)
		{
			delete[] m_children;
			m_children_num = 0;
		}

		if (this->m_level >= max_level)
		{
			return;
		}

		m_children_num = 4;
		m_children = new TileLodTree[m_children_num];
		for (int r = 0; r < 2; r++)
		{
			for (int c = 0; c < 2; c++)
			{
				int idx = r * 2 + c;
				TileLodTree& child = m_children[idx];
				child.m_child_index = idx;
				child.m_level = this->m_level + 1;
				child.m_tile_size = this->m_tile_size * 0.5;
				child.m_parent = this;
				child.m_ori = this->m_ori + osg::Vec2(
					child.m_tile_size * c,
					child.m_tile_size * r);
				child.m_gsd = this->m_gsd * 0.5;
				child.initChildren(max_level);
			}
		}
	}

	WorldRect2 TileLodTree::worldRect() const
	{
		return WorldRect2(m_ori.x(), m_ori.y(), m_tile_size, m_tile_size);
	}


	void printTileLodTree(TileLodTree* tree)
	{
		for (int i = 0; i < tree->level(); i++)
		{
			std::cout << "*";
		}
		std::cout << tree->getName() << std::endl;
		for (int i = 0; i < tree->childrenNum(); i++)
		{
			printTileLodTree(tree->child(i));
		}
	}
}
