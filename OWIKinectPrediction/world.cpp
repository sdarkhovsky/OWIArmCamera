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

void c_world::match_world_part(c_world_part* wp, const string& img_path) {
    size_t u, v, u1, v1, u2, v2;
    int i, j;
    vector <c_point_correspondence> map;
    vector <Vector2i> src_corner_pts;
    vector <Vector2i> tgt_corner_pts;

    if (world_parts.size() == 0)
        return;

    auto prev_wp = world_parts.back();

    for (u = 0; u < wp->point_cloud.points.size(); u++) {
        for (v = 0; v < wp->point_cloud.points[u].size(); v++) {
            if (wp->point_cloud.points[u][v].Clr_edge != 0) {
                if (wp->point_cloud.points[u][v].min_edge_corner_angle_cos > corner_angle_cosine_thresh)
                    src_corner_pts.push_back(Vector2i(u, v));
            }
        }
    }

    for (u = 0; u < prev_wp->point_cloud.points.size(); u++) {
        for (v = 0; v < prev_wp->point_cloud.points[u].size(); v++) {
            if (prev_wp->point_cloud.points[u][v].Clr_edge != 0) {
                if (prev_wp->point_cloud.points[u][v].min_edge_corner_angle_cos > corner_angle_cosine_thresh)
                    tgt_corner_pts.push_back(Vector2i(u, v));
            }
        }
    }

    const float angle_cos_tol = 1.0* 2.0 / 180.0;
    for (i = 0; i < src_corner_pts.size(); i++) {
        u1 = src_corner_pts[i](0);
        v1 = src_corner_pts[i](1);
        float src_angle_cos = wp->point_cloud.points[u1][v1].min_edge_corner_angle_cos;
        for (j = 0; j < tgt_corner_pts.size(); j++) {
            u2 = tgt_corner_pts[j](0);
            v2 = tgt_corner_pts[j](1);
            float tgt_angle_cos = prev_wp->point_cloud.points[u2][v2].min_edge_corner_angle_cos;
            if (abs(src_angle_cos - tgt_angle_cos) < angle_cos_tol) {
                map.push_back(c_point_correspondence(src_corner_pts[i], src_angle_cos, tgt_corner_pts[j], tgt_angle_cos));
            }
        }
    }

    if (img_path.size() > 0) {
        std::string file_path = img_path + ".correspondence.png";
        png_visualize_point_cloud_correspondence(file_path.c_str(), map, wp->point_cloud, prev_wp->point_cloud);
    }
}

void c_world::add_observation( c_point_cloud& point_cloud, double time, const string& img_path) {
    c_world_time world_time(time);

    c_world_part* wp = new c_world_part(point_cloud, world_time);
    match_world_part(wp, img_path);

    world_parts.push_back(wp);

    #if 1
    {
        if (img_path.size() > 0) {
            std::string file_path = img_path + ".detected_edge.xyze";

            c_kinect_image::write_file(file_path, wp->point_cloud, c_image_format::xyz);
        }
    }
    #endif
}

void c_world::predict() {
   
}

}