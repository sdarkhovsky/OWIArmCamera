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
    
void c_world::match_observed_scene_relation_to_existing_objects(c_observed_scene* scene, c_object_relation* relation) {
    for (auto wo = world_objects.begin(); wo != world_objects.end(); wo++) {
        for (auto obj_rel = wo->relations.begin(); obj_rel != wo->relations.end(); obj_rel++) {
            if (!(*obj_rel)->compatible(relation))
                continue;

            c_object_transformation transformation[2];
            for (int i = 0; i < 2; i++) {
                (*obj_rel)->calculate_transformation(relation, transformation[i], i);
            }

            c_object_point transformed_obj_point;
            auto pt = wo->points.begin();
            while (pt != wo->points.end()) {
                bool point_compatible_with_scene = false;
                for (int i = 0; i < 2; i++) {
                    transformation[i].transform_object_point(*pt, transformed_obj_point);
                    point_compatible_with_scene = point_compatible_with_scene || scene->compatible(transformed_obj_point);
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
}

void c_world::detect_object_in_scenes_from_transformation(c_observed_scene* scene, c_observed_scene* prev_scene, c_world_object& detected_object) {
    size_t prev_u, prev_v;
    int i, j;

    size_t prev_num_point_cloud_rows = prev_scene->point_cloud.points.size();
    size_t prev_num_point_cloud_cols = prev_scene->point_cloud.points[0].size();

    for (prev_u = 0; prev_u < prev_num_point_cloud_rows; prev_u++) {
        for (prev_v = 0; prev_v < prev_num_point_cloud_cols; prev_v++) {

            if (prev_scene->point_cloud.points[prev_u][prev_v].X == Vector3f::Zero())
                continue;

            if (prev_scene->point_cloud.points[prev_u][prev_v].object_assigned != 0)
                continue;

            c_object_point transformed_obj_point;
            c_object_point prev_point(prev_scene->point_cloud.points[prev_u][prev_v]);
            detected_object.transformation.transform_object_point(prev_point, transformed_obj_point);
            if (scene->compatible(transformed_obj_point)) {
                detected_object.points.push_back(prev_point);
            }
        }
    }
}

void c_world::match_observed_scene_relation_to_previous_scenes(c_observed_scene* scene, c_object_relation* relation) {
    size_t u, v, prev_u, prev_v;
    int i, j;

    vector <c_point_correspondence> map;
    vector <Vector2i> src_corner_pts;
    vector <Vector2i> tgt_corner_pts;

    auto prev_scene = observed_scenes.back();

    size_t prev_num_point_cloud_rows = prev_scene->point_cloud.points.size();
    size_t prev_num_point_cloud_cols = prev_scene->point_cloud.points[0].size();

    for (auto it = prev_scene->relations.begin(); it != prev_scene->relations.end(); it++) {

        if (!(*it)->compatible(relation))
            continue;

        c_object_transformation transformation;
        for (int i = 0; i < 2; i++) {
            (*it)->calculate_transformation(relation, transformation, i);

            if (transformation.is_identity)
                continue;

            list <c_object_relation*> object_relations;
            object_relations.push_back(*it);
            c_world_object detected_object(object_relations, transformation);
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


void c_world::match_observed_scene(c_observed_scene* scene) {
    size_t u, v, u1, v1, u2, v2;
    int i, j;

    if (observed_scenes.size() == 0)
        return;

    for (auto it = scene->relations.begin(); it != scene->relations.end(); it++) {

        match_observed_scene_relation_to_existing_objects(scene, *it);
        
        match_observed_scene_relation_to_previous_scenes(scene, *it);
    }
}

void c_world::add_observation( c_point_cloud& point_cloud, double time, string& img_path) {
    c_world_time world_time(time);

    c_observed_scene* scene = new c_observed_scene(point_cloud, world_time, img_path);

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
                c_kinect_image::write_file(file_path, scene->point_cloud, c_image_format::kinect);
            }
        }
    }
    #endif
    observed_scenes.push_back(scene);
}

void c_world::predict() {
   
}

}