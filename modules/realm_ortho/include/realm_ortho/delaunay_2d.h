

#ifndef PROJECT_DELAUNAY_2D_H
#define PROJECT_DELAUNAY_2D_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <realm_core/structs.h>
#include <realm_core/cv_grid_map.h>

namespace realm
{

class Delaunay2D
{
  public:
    using Ptr = std::shared_ptr<Delaunay2D>;
    using ConstPtr = std::shared_ptr<const Delaunay2D>;
  public:
    explicit Delaunay2D();
    std::vector<cv::Point2i> buildMesh(const CvGridMap &grid, const std::string &mask = "");

    void buildMeshWidthIndexs(const CvGridMap& grid, 
        size_t& vertics_num, size_t& face_num,
        std::vector<cv::Point2i>& vertics, 
        std::vector<size_t>& face_indexs,
        const std::string& mask = "");

};

} // namespace realm

#endif //PROJECT_DELAUNAY_2D_H
