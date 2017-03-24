#include "world.h"
#include "edge_feature.h"
#include "kinect_image.h"

namespace ais {

    c_observed_scene::c_observed_scene(c_point_cloud& _point_cloud, c_world_time& world_time, string& _img_path) {

        point_cloud = _point_cloud;
        time = world_time;

        img_path = _img_path;

        //  detect_color_edge_features_LOG(point_cloud);
        detect_color_edge_features_Canny(point_cloud);
        //    remove_edges_between_objects(point_cloud);
        //    find_edge_corners(point_cloud);
        //    calculate_boundaries(point_cloud);
        get_edge_chains(point_cloud, edge_chains);

        calculate_scene_relations();

        pcl_octree.add_points(point_cloud, octree_ind_to_uv);
    }

    void c_observed_scene::get_near_pnts(c_object_point& pnt, std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance) {
        pcl::PointXYZ searchPoint(pnt.X(0), pnt.X(1), pnt.X(2));
        int K = 8;
        pcl_octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    }

    bool c_observed_scene::compatible(c_object_point& pnt, bool mark_compatible_point) {

        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        float clr_diff_magnitude, min_clr_diff_magnitude = FLT_MAX;
        size_t u, v, min_u, min_v;
        get_near_pnts(pnt, pointIdxNKNSearch, pointNKNSquaredDistance);

        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {

            Vector2i& uv = octree_ind_to_uv[pointIdxNKNSearch[i]];

            u = uv(0);
            v = uv(1);

            clr_diff_magnitude = (point_cloud.points[u][v].Clr - pnt.Clr).norm();

            // std:cout << "i= " << i << " u=" << u << " v= " << v << " diff=" << clr_diff_magnitude << std::endl;

            if (clr_diff_magnitude < min_clr_diff_magnitude) {
                min_u = u;
                min_v = v;
                min_clr_diff_magnitude = clr_diff_magnitude;
            }
        }

        if (min_clr_diff_magnitude < compatible_point_color_difference_tolerance) {
            if (mark_compatible_point)
                point_cloud.points[min_u][min_v].object_assigned = true;
            return true;
        }

        return false;
    }

    bool c_observed_scene::calculate_edge_corner_relations() {
        size_t u, v, u1, v1, u2, v2;
        int i, j;

        size_t src_num_point_cloud_rows = point_cloud.points.size();
        size_t src_num_point_cloud_cols = point_cloud.points[0].size();

        for (u = 0; u < src_num_point_cloud_rows; u++) {
            for (v = 0; v < src_num_point_cloud_cols; v++) {
                if (point_cloud.points[u][v].object_assigned != 0)
                    continue;
                if (point_cloud.points[u][v].Clr_edge == 0)
                    continue;

                if (point_cloud.points[u][v].edge_corner_angle_cos != -1.0) {

#ifdef SMALL_KINS
                    //111111111111111111
                    if (img_path == "C:\\Projects\\OWIArmCamera\\KinectImages/img0.kin") {
                        if (!(u == 24 && v == 25)) {
                            continue;
                        }
                    }
                    if (img_path == "C:\\Projects\\OWIArmCamera\\KinectImages/img1_shoulder_1.kin") {
                        if (!(u == 23 && v == 31)) {
                            continue;
                        }
                    }
                    //11111111111111111
#endif
                //#define MEDIUM_KINS
#ifdef MEDIUM_KINS
                //111111111111111111
                    if (img_path == "C:\\Projects\\OWIArmCamera\\KinectImages/img0.kin") {
                        if (!(u == 197 && v == 701)) {
                            continue;
                        }
                    }
                    if (img_path == "C:\\Projects\\OWIArmCamera\\KinectImages/img1_shoulder_1.kin") {
                        if (!(u == 186 && v == 715)) {
                            continue;
                        }
                    }
                    //11111111111111111
#endif

                    c_corner_object_relation* relation = new c_corner_object_relation(c_object_point(point_cloud.points[u][v]),
                        point_cloud.points[u][v].edge_corner_angle_cos, point_cloud.points[u][v].edge_corner_dir1, point_cloud.points[u][v].edge_corner_dir2);

                    relations.push_back(relation);
                }
            }
        }

        return true;
    }

    bool c_observed_scene::calculate_edge_curvature_relations() {
        
        for (auto edge_chain_it = edge_chains.begin(); edge_chain_it != edge_chains.end(); edge_chain_it++) {
            auto& edge_chain = *edge_chain_it;
            size_t start_node = 0;
            float min_curvature = edge_chain[0].curvature;
            float max_curvature = min_curvature;
            int min_curvature_i = 0;
            int max_curvature_i = min_curvature_i;
            size_t node_i = 0;
            for (node_i = 1; node_i < edge_chain.size(); node_i++) {
                if (edge_chain[node_i].curvature > max_curvature) {
                    max_curvature = edge_chain[node_i].curvature;
                    max_curvature_i = node_i;
                }
                if (edge_chain[node_i].curvature < min_curvature) {
                    min_curvature = edge_chain[node_i].curvature;
                    min_curvature_i = node_i;
                }
                if (abs(edge_chain[node_i].curvature - edge_chain[start_node].curvature) > curvature_tolerance) {
                    c_curvature_object_relation* relation = new c_curvature_object_relation(point_cloud.points[edge_chain[start_node].uv(0)][edge_chain[start_node].uv(1)].X,
                        point_cloud.points[edge_chain[node_i].uv(0)][edge_chain[node_i].uv(1)].X,
                        edge_chain[start_node].normal, edge_chain[start_node].curvature);
                    relations.push_back(relation);
                    start_node = node_i;
                }
            }

            c_curvature_object_relation* relation = new c_curvature_object_relation(point_cloud.points[edge_chain[start_node].uv(0)][edge_chain[start_node].uv(1)].X,
                point_cloud.points[edge_chain[node_i-1].uv(0)][edge_chain[node_i-1].uv(1)].X,
                edge_chain[start_node].normal, edge_chain[start_node].curvature);
            relations.push_back(relation);
        }

        return true;
    }

    bool c_observed_scene::calculate_scene_relations() {
 //       calculate_edge_corner_relations();
        calculate_edge_curvature_relations();

        return true;
    }
}