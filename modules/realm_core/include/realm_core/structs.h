

#ifndef PROJECT_STRUCTS_H
#define PROJECT_STRUCTS_H

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <realm_core/utm32.h>
#include <realm_core/camera.h>

namespace realm
{

/*!
 * @brief Basic struct for definition of a plane. Might be replaced with a simpler definition in the future (raw arrays)
 */
struct Plane
{
    using Ptr = std::shared_ptr<Plane>;
    using ConstPtr = std::shared_ptr<const Plane>;
    cv::Mat pt;
    cv::Mat n;
};

/*!
 * @brief Basic struct for parameters additionally providing a help description
 * @tparam T Type of the parameter value
 */
template <typename T>
struct Parameter_t
{
    T value;
    std::string help;
};

/*!
 * @brief Basic struct for a face of a mesh
 */
struct Face
{
  cv::Point3d vertices[3];
  cv::Vec4b color[3];
};


class Mesh
{
public:
	using Ptr = std::shared_ptr<Mesh>;
	using ConstPtr = std::shared_ptr<const Mesh>;

    inline Mesh(size_t _num_vert, size_t _num_face)
    {
        this->resize(_num_vert, _num_face);
		m_texture = cv::Mat::zeros(0, 0, CV_8UC4);
    }

	inline Mesh() :m_num_vert(0), m_num_face(0),
		m_vertices(0), m_texCoords(0), m_colors(0), m_faces(0) {
		m_texture = cv::Mat::zeros(0, 0, CV_8UC4);
	}

    ~Mesh() { this->clear(); m_texture.release(); }

	inline void clear()
	{
		if (m_num_face > 0)
		{
			delete[] m_faces;
			m_num_face = 0;
		}

		if (m_num_vert > 0)
		{
			delete[] m_vertices;
			delete[] m_colors;
			delete[] m_texCoords;
			m_num_vert = 0;
		}

		m_texture = cv::Mat::zeros(0, 0, CV_8UC4);
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
			m_vertices = new cv::Point3d[m_num_vert];
			m_colors = new cv::Vec4b[m_num_vert];
			m_texCoords = new cv::Vec2f[m_num_vert];
		}
		else
		{
			m_vertices = NULL;
			m_colors = NULL;
			m_texCoords = NULL;
		}

		if (m_num_face > 0)
		{
			m_faces = new size_t[m_num_face * 3];
		}
		else
		{
			m_faces = NULL;
		}
	}

	inline void setVert(size_t vert_index, const cv::Point3d& point, const cv::Vec4b& color)
	{
		assert(vert_index < m_num_vert);
		m_vertices[vert_index] = point;
		m_colors[vert_index] = color;
	}

	inline void setFace(size_t face_index, size_t vert_0, size_t vert_1, size_t vert_2)
	{
		assert(face_index < m_num_face);
		assert(vert_0 < m_num_vert);
		assert(vert_1 < m_num_vert);
		assert(vert_2 < m_num_vert);
		m_faces[face_index * 3 + 0] = vert_0;
		m_faces[face_index * 3 + 1] = vert_1;
		m_faces[face_index * 3 + 2] = vert_2;
	}

	inline void getVert(size_t vert_index, cv::Point3d& point, cv::Vec4b& color)
	{
		point = m_vertices[vert_index];
		color = m_colors[vert_index];
	}

	inline void getFace(size_t face_index, size_t& vert_0, size_t& vert_1, size_t& vert_2)
	{
		vert_0 = m_faces[face_index * 3 + 0];
		vert_1 = m_faces[face_index * 3 + 1];
		vert_2 = m_faces[face_index * 3 + 2];
	}

	inline cv::Point3d* vertices() { return m_vertices; }

	inline cv::Vec4b* colors() { return m_colors; }

	inline cv::Vec2f* texCoords() { return m_texCoords; }

	inline size_t* faces() { return m_faces; }

	inline cv::Mat& texture() { return m_texture; }

private:
	size_t m_num_vert;
	size_t m_num_face;
	cv::Point3d* m_vertices;
	cv::Vec2f* m_texCoords;
	cv::Vec4b* m_colors;
	cv::Mat m_texture;
	size_t* m_faces;

};

}

#endif //PROJECT_STRUCTS_H
