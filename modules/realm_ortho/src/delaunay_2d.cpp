

#include <realm_ortho/delaunay_2d.h>

using namespace realm;

Delaunay2D::Delaunay2D()
{

}

std::vector<cv::Point2i> Delaunay2D::buildMesh(const CvGridMap &grid, const std::string &mask)
{
  //CGAL boilerplate
  typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
  //We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
  typedef CGAL::Triangulation_vertex_base_with_info_2<cv::Point2i, Kernel> VertexBase;
  typedef CGAL::Constrained_triangulation_face_base_2<Kernel> FaceBase;
  typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBase> TriangulationDataStruct;
  typedef CGAL::Delaunay_triangulation_2<Kernel, TriangulationDataStruct> DelaunayTriangulation;
  typedef DelaunayTriangulation::Point CGALPoint;

  cv::Mat valid;
  if (!mask.empty() && grid.exists(mask))
  {
      valid = grid[mask];
  }
  else
  {
      valid = cv::Mat::ones(grid.size(), CV_8UC1) * 255;
  }

  std::vector< std::pair<CGALPoint, cv::Point2i>> pts;
  pts.reserve(static_cast<size_t>(valid.rows * valid.cols));

  // Create and add vertices
  for (uint32_t r = 0; r < valid.rows; ++r)
    for (uint32_t c = 0; c < valid.cols; ++c)
      if (valid.at<uchar>(r, c) > 0)
      {
        cv::Point2d pt = grid.atPosition2d(r, c);
        pts.push_back(std::make_pair(CGALPoint(pt.x, pt.y), cv::Point2i(c, r)));
      }

  // The DT is built
  DelaunayTriangulation dt(pts.begin(), pts.end());

  std::vector<cv::Point2i> vertex_ids;
  vertex_ids.reserve(dt.number_of_faces()*3);
  if (vertex_ids.capacity() > 0)
  {
    for (auto it = dt.faces_begin(); it != dt.faces_end(); ++it)
      for (int i = 0; i < 3; ++i)
        vertex_ids.push_back(it->vertex(i)->info());
  }

  return vertex_ids;
}

void realm::Delaunay2D::buildMeshWidthIndexs(const CvGridMap& grid, 
    size_t& vertics_num, size_t& face_num,
    std::vector<cv::Point2i>& vertics, 
    std::vector<size_t>& face_indexs, const std::string& mask)
{
    //CGAL boilerplate
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    //We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
    typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, Kernel> VertexBase;
    typedef CGAL::Constrained_triangulation_face_base_2<Kernel> FaceBase;
    typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBase> TriangulationDataStruct;
    typedef CGAL::Delaunay_triangulation_2<Kernel, TriangulationDataStruct> DelaunayTriangulation;
    typedef DelaunayTriangulation::Point CGALPoint;

    cv::Mat valid;
    if (!mask.empty() && grid.exists(mask))
    {
        valid = grid[mask];
    }
    else
    {
        valid = cv::Mat::ones(grid.size(), CV_8UC1) * 255;
    }

    std::vector< std::pair<CGALPoint, size_t>> pts;
    pts.reserve(static_cast<size_t>(valid.rows * valid.cols));

    size_t vert_count = 0;
    // Create and add vertices
    for (uint32_t r = 0; r < valid.rows; ++r)
    {
        for (uint32_t c = 0; c < valid.cols; ++c)
        {
            if (valid.at<uchar>(r, c) > 0)
            {
                cv::Point2d pt = grid.atPosition2d(r, c);
                vertics.push_back(cv::Point2i(c, r));
                pts.push_back(std::make_pair(CGALPoint(pt.x, pt.y), vert_count));
                vert_count++;
            }
        }
    }

    vertics_num = vert_count;

    // The DT is built
    DelaunayTriangulation dt(pts.begin(), pts.end());

    face_num = dt.number_of_faces();

    face_indexs.reserve(face_num * 3);
    if (face_num > 0)
    {
        for (auto it = dt.faces_begin(); it != dt.faces_end(); ++it)
        {
            for (int i = 0; i < 3; ++i)
            {
                size_t vt = it->vertex(i)->info();
                face_indexs.push_back(vt);
            }
        }
    }

}
