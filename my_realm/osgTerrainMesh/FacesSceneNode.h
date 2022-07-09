#ifndef MYREALM2_FACESSCENENODE_H
#define MYREALM2_FACESSCENENODE_H

#include <cassert>
#include <memory>
#include <map>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Callback>
#include <OpenThreads/Thread>
#include <OpenThreads/Mutex>

namespace MyREALM2
{

	class Mesh
	{
	public:
		using Ptr = std::shared_ptr<Mesh>;
		using ConstPtr = std::shared_ptr<const Mesh>;

		inline Mesh(size_t _num_vert, size_t _num_face)
		{
			this->resize(_num_vert, _num_face);
		}

		inline Mesh() :m_num_vert(0), m_num_face(0)
		{
			m_vertices = new osg::Vec3Array;
			m_colors = new osg::Vec4Array;
			m_texCoords = new osg::Vec2Array;
			m_faces = new osg::UIntArray;
		}

		~Mesh()
		{
		}

		inline void clear()
		{
			if (m_num_face > 0)
			{
				m_faces->clear();
				m_num_face = 0;
			}

			if (m_num_vert > 0)
			{
				m_vertices->clear();
				m_colors->clear();
				m_texCoords->clear();
				m_num_vert = 0;
			}
		}

		inline size_t vert_size() { return m_num_vert; }

		inline size_t face_size() { return m_num_face; }

		inline void resize(size_t _num_vert, size_t _num_face)
		{
			this->clear();
			m_num_vert = _num_vert;
			m_num_face = _num_face;
			if (m_num_vert > 0)
			{
				m_vertices->resize(m_num_vert);
				m_colors->resize(m_num_vert);
				m_texCoords->resize(m_num_vert);
			}
			else
			{
				m_vertices->clear();
				m_colors->clear();
				m_texCoords->clear();
			}

			if (m_num_face > 0)
			{
				m_faces->resize(m_num_face * 3);
			}
			else
			{
				m_faces->clear();
			}
		}

		inline void setVert(size_t vert_index, const osg::Vec3& point, const osg::Vec4& color)
		{
			assert(vert_index < m_num_vert);
			m_vertices->at(vert_index) = point;
			m_colors->at(vert_index) = color;
		}

		inline void setFace(size_t face_index, size_t vert_0, size_t vert_1, size_t vert_2)
		{
			assert(face_index < m_num_face);
			assert(vert_0 < m_num_vert);
			assert(vert_1 < m_num_vert);
			assert(vert_2 < m_num_vert);
			m_faces->at(face_index * 3 + 0) = vert_0;
			m_faces->at(face_index * 3 + 1) = vert_1;
			m_faces->at(face_index * 3 + 2) = vert_2;
		}

		inline void getVert(size_t vert_index, osg::Vec3& point, osg::Vec4& color)
		{
			point = m_vertices->at(vert_index);
			color = m_colors->at(vert_index);
		}

		inline void getFace(size_t face_index, size_t& vert_0, size_t& vert_1, size_t& vert_2)
		{
			vert_0 = m_faces->at(face_index * 3 + 0);
			vert_1 = m_faces->at(face_index * 3 + 1);
			vert_2 = m_faces->at(face_index * 3 + 2);
		}

		inline osg::Vec3Array* vertices() { return m_vertices.get(); }

		inline osg::Vec4Array* colors() { return m_colors.get(); }

		inline osg::UIntArray* faces() { return m_faces.get(); }

		inline osg::Vec2Array* texCoords() { return m_texCoords.get(); }

	private:
		size_t m_num_vert;
		size_t m_num_face;
		osg::ref_ptr<osg::Vec3Array> m_vertices;
		osg::ref_ptr<osg::Vec4Array> m_colors;
		osg::ref_ptr<osg::Vec2Array> m_texCoords;
		osg::ref_ptr<osg::UIntArray> m_faces;
	};

	class FacesReceiverThread : public OpenThreads::Thread
	{
	public:
		FacesReceiverThread();

		virtual int cancel();
		virtual void run();

		void updateMesh();

		void setVerticsAndColors(const osg::Vec3Array& arr, const osg::Vec4Array& color_arr,
			const osg::UIntArray& index_arr);

		bool getVerticsAndColors(osg::Vec3Array& arr, osg::Vec4Array& color_arr,
			osg::Vec2Array& texCoords_arr,
			osg::UIntArray& index_arr);

	protected:
		OpenThreads::Mutex _mutex;
		/*osg::ref_ptr<osg::Vec3Array> _lineVertics;
		osg::ref_ptr<osg::Vec4Array> _colors;
		osg::ref_ptr<osg::UIntArray> _indexs;*/

		Mesh::Ptr _mesh;
		int _update_it;

		bool _done;
		bool _dirty;
	};


	class FacesDrawCallback :public osg::DrawableUpdateCallback
	{
	public:
		FacesDrawCallback(FacesReceiverThread* p_thread);
		~FacesDrawCallback();

		virtual void update(osg::NodeVisitor* nv, osg::Drawable* drawable) override;

	protected:
		FacesReceiverThread* m_thread;
	};

	class TileFacesNode : public  osg::Referenced
	{
	public:
		TileFacesNode();

		~TileFacesNode();

		const osg::BoundingBoxd& getBoundingBox() const;

		FacesReceiverThread* facesThread();

		osg::ref_ptr<osg::Geode> facesGeode();

		static osg::ref_ptr<TileFacesNode> createTileFacesNode(
			int row, int col,
			double tile_size);
		
	private:
		osg::BoundingBoxd m_bound;
		int m_row;
		int m_col;
		osg::ref_ptr<osg::Geode> m_geode;
		osg::ref_ptr<FacesDrawCallback> m_facesCLB;
		FacesReceiverThread* m_facesThread;
	};



	typedef std::map<std::string, osg::ref_ptr<TileFacesNode>>  TileFacesNodeMap;

	osg::ref_ptr<osg::Geode> createFacesGeode(FacesDrawCallback* clb);

}

#endif  // !MYREALM2_FACESSCENENODE_H