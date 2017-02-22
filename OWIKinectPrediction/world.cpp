#include "world.h"
#include "edge_feature.h"
#include "plane_feature.h"
#include "kinect_image.h"

#include "png_visualize.h"

bool get_executable_file_path(char* file_path, int file_path_length);

namespace ais {
    
c_world_time::c_world_time(double _time) {
    time = _time;
}
    
c_world_part::c_world_part(c_point_cloud& _point_cloud, c_world_time& world_time) {

    point_cloud = _point_cloud;
    time = world_time;

//    bool result = detect_color_edge_features_LOG(point_cloud);
    bool result = detect_color_edge_features_Canny(point_cloud);
    result = find_edge_corners(point_cloud);
}

void c_world::match_world_part(c_world_part& wp, string& img_path) {
    size_t u, v, u1, v1, u2, v2;
    int i, j;
    vector <c_point_correspondence> map;
    vector <Vector2i> src_corner_pts;
    vector <Vector2i> tgt_corner_pts;

    if (world_parts.size() == 0)
        return;

    auto& prev_wp = world_parts.back();

    size_t src_num_point_cloud_rows = wp.point_cloud.points.size();
    size_t src_num_point_cloud_cols = wp.point_cloud.points[0].size();
    size_t tgt_num_point_cloud_rows = prev_wp.point_cloud.points.size();
    size_t tgt_num_point_cloud_cols = prev_wp.point_cloud.points[0].size();

    for (u = 0; u < src_num_point_cloud_rows; u++) {
        for (v = 0; v < src_num_point_cloud_cols; v++) {
            if (wp.point_cloud.points[u][v].Clr_edge != 0) {
                if (wp.point_cloud.points[u][v].edge_corner_angle_cos > corner_angle_cosine_thresh)
                    src_corner_pts.push_back(Vector2i(u, v));
            }
        }
    }

    for (u = 0; u < tgt_num_point_cloud_rows; u++) {
        for (v = 0; v < tgt_num_point_cloud_cols; v++) {
            if (prev_wp.point_cloud.points[u][v].Clr_edge != 0) {
                if (prev_wp.point_cloud.points[u][v].edge_corner_angle_cos > corner_angle_cosine_thresh)
                    tgt_corner_pts.push_back(Vector2i(u, v));
            }
        }
    }

    const float angle_cos_tol = 3.0* 2.0 / 180.0;
    for (i = 0; i < src_corner_pts.size(); i++) {
        u1 = src_corner_pts[i](0);
        v1 = src_corner_pts[i](1);
        float src_angle_cos = wp.point_cloud.points[u1][v1].edge_corner_angle_cos;

        //111111111111111111
        if (!((v1 == 1006 && u1 == 608) || (v1 == 1006 && u1 == 608))) {
            continue;
        }
        //11111111111111111

        for (j = 0; j < tgt_corner_pts.size(); j++) {
            u2 = tgt_corner_pts[j](0);
            v2 = tgt_corner_pts[j](1);
            float tgt_angle_cos = prev_wp.point_cloud.points[u2][v2].edge_corner_angle_cos;
            //111111111111111111
            if (!((v2 == 993 && u2 == 604) || (v2 == 994 && u2 == 603))) {
                continue;
            }
            //11111111111111111

            if (abs(src_angle_cos - tgt_angle_cos) < angle_cos_tol) {
                map.push_back(c_point_correspondence(src_corner_pts[i], tgt_corner_pts[j]));
            }
        }
    }

    if (img_path.size() > 0) {
        std::string file_path = img_path + ".correspondence.png";
        png_visualize_point_cloud_correspondence(file_path.c_str(), map, wp.point_cloud, prev_wp.point_cloud);
    }

    for (auto it = map.begin(); it != map.end(); it++) {
        c_rigid_body_transformation transformation;
        c_world_part out_wp;

        for (int switch_dir_mapping = 0; switch_dir_mapping < 1; switch_dir_mapping++) {
            if (calculate_world_part_transformation(wp, prev_wp, *it, transformation, switch_dir_mapping)) {
                if (estimate_part_points_from_transformation(wp, prev_wp, transformation, out_wp))
                    break;
            }
        }
    }
}

bool c_world::compare_with_world_part(Vector3f X, Vector3f Clr, c_world_part& wp) {
    return true;
}


bool c_world::estimate_part_points_from_transformation(c_world_part& wp, c_world_part& prev_wp, c_rigid_body_transformation& rbt, c_world_part& out_wp) {
    size_t u, v;
    Vector3f X;

    size_t src_num_point_cloud_rows = wp.point_cloud.points.size();
    size_t src_num_point_cloud_cols = wp.point_cloud.points[0].size();
    size_t tgt_num_point_cloud_rows = prev_wp.point_cloud.points.size();
    size_t tgt_num_point_cloud_cols = prev_wp.point_cloud.points[0].size();

    for (u = 0; u < src_num_point_cloud_rows; u++) {
        for (v = 0; v < src_num_point_cloud_cols; v++) {
            X = rbt.rotation*(wp.point_cloud.points[u][v].X + rbt.translation);

            if (compare_with_world_part(X, wp.point_cloud.points[u][v].Clr, prev_wp)) {

            }
            else {

            }
        }
    }
    return true;
}

bool c_world::calculate_world_part_transformation(c_world_part& wp, c_world_part& prev_wp, c_point_correspondence& map, c_rigid_body_transformation& rbt, int switch_dir_mapping) {
    Vector3f tgt_dir1, tgt_dir2, src_dir1, src_dir2, tmp;
    rbt.translation = prev_wp.point_cloud.points[map.tgt_uv(0)][map.tgt_uv(1)].X - wp.point_cloud.points[map.src_uv(0)][map.src_uv(1)].X;
    
    tgt_dir1 = prev_wp.point_cloud.points[map.tgt_uv(0)][map.tgt_uv(1)].edge_corner_dir1;
    tgt_dir2 = prev_wp.point_cloud.points[map.tgt_uv(0)][map.tgt_uv(1)].edge_corner_dir2;

    src_dir1 = prev_wp.point_cloud.points[map.src_uv(0)][map.src_uv(1)].edge_corner_dir1;
    src_dir2 = prev_wp.point_cloud.points[map.src_uv(0)][map.src_uv(1)].edge_corner_dir2;

    if (switch_dir_mapping) {
        tmp = src_dir1;
        src_dir2 = src_dir1;
        src_dir1 = tmp;
    } 

    Matrix3f src_M, tgt_M; 
    src_M << src_dir1, src_dir2, src_dir1.cross(src_dir2);
    tgt_M << tgt_dir1, tgt_dir2, tgt_dir1.cross(tgt_dir2);
    rbt.rotation = src_M.inverse()*tgt_M;

    return true;
}

void c_world::add_observation( c_point_cloud& point_cloud, double time, string& img_path) {
    c_world_time world_time(time);

    c_world_part wp(point_cloud, world_time);
    match_world_part(wp, img_path);

    #if 1
    {
        if (img_path.size() > 0) {
            std::string file_path = img_path + ".detected_edge.xyze";

            c_kinect_image::write_file(file_path, wp.point_cloud, c_image_format::xyz);
        }
    }
    #endif
    world_parts.push_back(std::move(wp));
}

void c_world::predict() {
   
}

}