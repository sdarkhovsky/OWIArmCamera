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
    
c_observed_scene::c_observed_scene(c_point_cloud& _point_cloud, c_world_time& world_time, string& _img_path) {

    point_cloud = _point_cloud;
    time = world_time;

    img_path = _img_path;

//  detect_color_edge_features_LOG(point_cloud);
    detect_color_edge_features_Canny(point_cloud);
//    remove_edges_between_objects(point_cloud);
//    find_edge_corners(point_cloud);
//    calculate_boundaries(point_cloud);
    get_edge_segments(point_cloud);
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

bool c_observed_scene::calculate_scene_relations() {
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

                c_object_relation relation(c_object_point(point_cloud.points[u][v]),
                    point_cloud.points[u][v].edge_corner_angle_cos, point_cloud.points[u][v].edge_corner_dir1, point_cloud.points[u][v].edge_corner_dir2);

                relations.push_back(relation);
            }
        }
    }

    return true;
}

bool c_object_relation::calculate_transformation(c_object_relation& tgt_relation, c_object_transformation& transformation, bool switch_dir_mapping) {

    transformation.translation_before_rotation = - pnt.X;
    transformation.translation_after_rotation = tgt_relation.pnt.X;

    Vector3f src_dir1 = dir1;
    Vector3f src_dir2 = dir2;

    if (switch_dir_mapping) {
        src_dir1 = dir2;
        src_dir2 = dir1;
    }

    Matrix3f src_M, tgt_M;
    src_M << src_dir1, src_dir2, src_dir1.cross(src_dir2);
    tgt_M << tgt_relation.dir1, tgt_relation.dir2, tgt_relation.dir1.cross(tgt_relation.dir2);
    // for the rotation matrix to be truly orthogonal the angles in the realations must be the same
    transformation.rotation = tgt_M*src_M.inverse();

    transformation.is_identity = ((transformation.translation_before_rotation + transformation.translation_after_rotation).norm() < translation_zero_tolerance) &&
        ((transformation.rotation - Matrix3f::Identity()).norm() < distance_to_identity_matrix_tolerance);
    return true;
}

void c_world::match_observed_scene_relation_to_existing_objects(c_observed_scene& scene, c_object_relation& relation) {
    for (auto wo = world_objects.begin(); wo != world_objects.end(); wo++) {
        if (!wo->relation.compatible(relation))
            continue;

        c_object_transformation transformation[2];
        for (int i = 0; i < 2; i++) {
            wo->relation.calculate_transformation(relation, transformation[i], i);
        }

        c_object_point transformed_obj_point;
        auto pt = wo->points.begin();
        while (pt != wo->points.end()) {
            bool point_compatible_with_scene = false;
            for (int i = 0; i < 2; i++) {
                transformation[i].transform_object_point(*pt, transformed_obj_point);
                point_compatible_with_scene = point_compatible_with_scene || scene.compatible(transformed_obj_point);
            }

            if (point_compatible_with_scene) {
                pt++;
                continue;
            }
            else {
                pt = wo->points.erase(pt);
                continue;
            }
        }
    }
}

void c_world::detect_object_in_scenes_from_transformation(c_observed_scene& scene, c_observed_scene& prev_scene, c_world_object& detected_object) {
    size_t prev_u, prev_v;
    int i, j;

    size_t prev_num_point_cloud_rows = prev_scene.point_cloud.points.size();
    size_t prev_num_point_cloud_cols = prev_scene.point_cloud.points[0].size();

    for (prev_u = 0; prev_u < prev_num_point_cloud_rows; prev_u++) {
        for (prev_v = 0; prev_v < prev_num_point_cloud_cols; prev_v++) {

            if (prev_scene.point_cloud.points[prev_u][prev_v].X == Vector3f::Zero()) 
                continue;

            if (prev_scene.point_cloud.points[prev_u][prev_v].object_assigned != 0)
                continue;

            c_object_point transformed_obj_point;
            c_object_point prev_point(prev_scene.point_cloud.points[prev_u][prev_v]);
            detected_object.transformation.transform_object_point(prev_point, transformed_obj_point);
            if (scene.compatible(transformed_obj_point)) {
                detected_object.points.push_back(prev_point);
            }
        }
    }
}

void c_world::match_observed_scene_relation_to_previous_scenes(c_observed_scene& scene, c_object_relation& relation) {
    size_t u, v, prev_u, prev_v;
    int i, j;

    vector <c_point_correspondence> map;
    vector <Vector2i> src_corner_pts;
    vector <Vector2i> tgt_corner_pts;

    auto& prev_scene = observed_scenes.back();

    size_t prev_num_point_cloud_rows = prev_scene.point_cloud.points.size();
    size_t prev_num_point_cloud_cols = prev_scene.point_cloud.points[0].size();

    for (auto it = prev_scene.relations.begin(); it != prev_scene.relations.end(); it++) {

        if (!(*it).compatible(relation))
            continue;

        c_object_transformation transformation;
        for (int i = 0; i < 2; i++) {
            (*it).calculate_transformation(relation, transformation, i);

            if (transformation.is_identity)
                continue;

            c_world_object detected_object(*it, transformation);
            detect_object_in_scenes_from_transformation(scene, prev_scene, detected_object);
            if (detected_object.points.size() > 0) {
                world_objects.push_back(detected_object);
            }
        }
    }

#if VISUALIZE_CORRESPONDENCE

    for (j = 0; j < tgt_corner_pts.size(); j++) {
        u2 = tgt_corner_pts[j](0);
        v2 = tgt_corner_pts[j](1);
        float tgt_angle_cos = prev_wp.point_cloud.points[u2][v2].edge_corner_angle_cos;

        if (abs(src_angle_cos - tgt_angle_cos) < angle_cos_tol) {
            map.push_back(c_point_correspondence(src_corner_pts[i], tgt_corner_pts[j]));
        }
    }

    if (img_path.size() > 0) {
        std::string file_path = img_path + ".correspondence.png";
        png_visualize_point_cloud_correspondence(file_path.c_str(), map, wp.point_cloud, prev_wp.point_cloud);
    }
#endif
}


void c_world::match_observed_scene(c_observed_scene& scene) {
    size_t u, v, u1, v1, u2, v2;
    int i, j;

    if (observed_scenes.size() == 0)
        return;

    for (auto it = scene.relations.begin(); it != scene.relations.end(); it++) {

        match_observed_scene_relation_to_existing_objects(scene, *it);
        
        match_observed_scene_relation_to_previous_scenes(scene, *it);
    }
}

void c_world::add_observation( c_point_cloud& point_cloud, double time, string& img_path) {
    c_world_time world_time(time);

    c_observed_scene scene(point_cloud, world_time, img_path);

//#define MATCH_OBSERVED_SCENE
#ifdef MATCH_OBSERVED_SCENE
    match_observed_scene(scene);
#endif

    #if 1
    {
        if (img_path.size() > 0) {
            std::size_t found = img_path.find_last_of("/");
            if (found != string::npos) {
                std::string file_path = img_path.substr(0, found) + "\\detected" + img_path.substr(found);
                c_kinect_image::write_file(file_path, scene.point_cloud, c_image_format::kinect);
            }
        }
    }
    #endif
    observed_scenes.push_back(std::move(scene));
}

void c_world::predict() {
   
}

}