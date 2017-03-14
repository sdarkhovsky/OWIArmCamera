#ifndef EDGE_FEATURE_H
#define EDGE_FEATURE_H

#include <vector>

namespace ais {

    bool detect_edge_features(c_point_cloud& point_cloud);
    bool detect_color_edge_features_LOG(c_point_cloud& point_cloud);
    bool detect_color_edge_features_Canny(c_point_cloud& point_cloud);
    bool find_edge_corners(c_point_cloud& point_cloud);
    bool remove_edges_between_objects(c_point_cloud& point_cloud);
    bool calculate_boundaries(c_point_cloud& point_cloud);
    bool calculate_gaussian_mask(vector<vector<float>>& mask, float sigma, int mask_radius);
    bool get_edge_chains(c_point_cloud& point_cloud, list<list<Vector2i>>& edge_chains);
    bool mark_edge_chains(c_point_cloud& point_cloud, list<list<Vector2i>>& edge_chains);
    bool get_edge_segments(c_point_cloud& point_cloud, list<list<Vector2i>>& edge_chains);


    class c_edge_feature {
    public:
//        Vector3f normal;
    };
}

#endif