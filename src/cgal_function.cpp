// #include"cgal_function.h"


//    //参数设置
//     // const double sharpness_angle = 25;   // control sharpness of the result.
//     // const double edge_sensitivity = 0;    // higher values will sample more points near the edges
//     // const double neighbor_radius = 0.25;  // initial size of neighborhood.
//     // const std::size_t number_of_output_points = points.size() * 8;
// std::vector<PointVectorPair>cgal_unsampling(std::vector<PointVectorPair> points,double sharpness_angle,double edge_sensitivity,
//                                           double neighbor_radius, int number_of_output_points )
// {
//     //std::vector<PointVectorPair> out_points;
//     CGAL::edge_aware_upsample_point_set<Concurrency_tag>(
//         points,std::back_inserter(points),
//         CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
//         normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).
//         sharpness_angle(sharpness_angle).
//         edge_sensitivity(edge_sensitivity).
//         neighbor_radius(neighbor_radius).
//         number_of_output_points(number_of_output_points));
//     return points;
// }