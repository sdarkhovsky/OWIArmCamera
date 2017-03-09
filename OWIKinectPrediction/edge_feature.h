#ifndef EDGE_FEATURE_H
#define EDGE_FEATURE_H

namespace ais {

    bool detect_edge_features(c_point_cloud& point_cloud);
    bool detect_color_edge_features_LOG(c_point_cloud& point_cloud);
    bool detect_color_edge_features_Canny(c_point_cloud& point_cloud);
    bool find_edge_corners(c_point_cloud& point_cloud);
    bool remove_edges_between_objects(c_point_cloud& point_cloud);

    class c_edge_feature {
    public:
//        Vector3f normal;
    };
}

#endif