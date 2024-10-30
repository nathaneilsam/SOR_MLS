// #ifndef CGAL_FUNCTION_H
// #define CGAL_FUNCTION_H


// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/edge_aware_upsample_point_set.h>
// // #include <CGAL/IO/read_points.h>
// #include<CGAL/IO/read_ply_points.h>
// #include<CGAL/IO/write_ply_points.h>
// #include<CGAL/Point_set_3.h>
// // #include <CGAL/draw_point_set_3.h>
// // #include <CGAL/IO/write_points.h>
#include <string>
#include <vector>
#include <fstream>



// typedef CGAL::Simple_cartesian<double> Kernel;
// typedef Kernel::Point_3 CGAL_Point;
// typedef Kernel::Vector_3 Vector;
// struct PointWithAttributes {
//     CGAL_Point point; // 点的坐标
//     float intensity;        // 点的强度
//     int label;             // 点的标签
//         // 构造函数
//     PointWithAttributes(const CGAL_Point& p, float i, int l)
//         : point(p), intensity(i), label(l) {}
// };

// typedef std::pair<CGAL_Point, Vector> PointVectorPair;
// typedef CGAL::Parallel_tag Concurrency_tag;
// // typedef std::pair<PointWithAttributes, Vector> PointVectorPair;
// typedef std::vector<PointWithAttributes> PointWithAttributesVector;


// /// @brief    //参数设置
// /// @param points 
// /// @param sharpness_angle  = 25;   // control sharpness of the result.
// /// @param edge_sensitivity  = 0;    // higher values will sample more points near the edges
// /// @param neighbor_radius  = 0.25;  // initial size of neighborhood.
// /// @param number_of_output_points   = points.size() * 8;
// /// @return 
// std::vector<PointVectorPair>cgal_unsampling(std::vector<PointVectorPair> points,double sharpness_angle,double edge_sensitivity,
//                                           double neighbor_radius, int number_of_output_points );
// // pcl::PointCloud<pcl::Normal>::Ptr pcl_feature_normals_k(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in,float K);
// // pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_base_cloudwithnormals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals);
// #endif // CGAL_FUNCTION_H

