#ifndef EDGE_FEATURE_H
#define EDGE_FEATURE_H

#include <vector>
#include <map>

namespace ais {

    bool detect_edge_features(c_point_cloud& point_cloud);
    bool detect_color_edge_features_LOG(c_point_cloud& point_cloud);
    bool detect_color_edge_features_Canny(c_point_cloud& point_cloud);
    bool find_edge_corners(c_point_cloud& point_cloud);
    bool remove_edges_between_objects(c_point_cloud& point_cloud);
    bool calculate_boundaries(c_point_cloud& point_cloud);
    bool calculate_gaussian_mask(vector<vector<float>>& mask, float sigma, int mask_radius);
    bool calculate_edge_chain_curvature(c_point_cloud& point_cloud, vector<c_edge_node>& edge_chain);
    bool get_edge_chains(c_point_cloud& point_cloud, vector<vector<c_edge_node>>& edge_chains);
}

#endif