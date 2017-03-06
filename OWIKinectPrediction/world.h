#ifndef WORLD_H
#define WORLD_H

#include "point_cloud.h"

#include <vector>
#include <string>
#include <memory>

#include "pcl_octree.h"

namespace ais {
class c_world_time {
public:    
    c_world_time() {
        time = 0;
    };
    c_world_time(double time);
    
    double time;
};

class c_object_point {
public:
    c_object_point() {}
    c_object_point(c_point_cloud_point& point_cloud_point) {
        X = point_cloud_point.X;
        Clr = point_cloud_point.Clr;
    }

    Vector3f X;
    Vector3f Clr;
};

class c_object_transformation {
public:
    void transform_object_point(c_object_point& in, c_object_point& out) {
        out = in;
        out.X = rotation * in.X + translation;
    }

    Matrix3f rotation;
    Vector3f translation;
};

class c_object_relation {
public:
    c_object_relation(c_object_point& _pnt, float _angle_cos_value, Vector3f& _dir1, Vector3f& _dir2) {
        pnt = _pnt;
        angle_cos_value = _angle_cos_value;
        dir1 = _dir1;
        dir2 = _dir2;
    }
//    c_object_relation(c_object_relation& object_relation) = default;
    c_object_relation(const c_object_relation& object_relation) = default;
 //   c_object_relation& operator=(c_object_relation& object_relation) = default;

    const float angle_cos_tolerance = 3.0* 2.0 / 180.0;
    bool compatible(c_object_relation& relation) {
        return (abs(angle_cos_value - relation.angle_cos_value) < angle_cos_tolerance);
    }
    bool calculate_transformation(c_object_relation& tgt_relation, c_object_transformation& transf, bool switch_dir_mapping);
    c_object_point pnt;
    float angle_cos_value;
    Vector3f dir1;
    Vector3f dir2;
};

class c_world_object {
public:
    c_world_object(c_object_relation& _relation, c_object_transformation& _transformation) : relation(_relation), transformation(_transformation) {
    }

    c_object_relation relation;
    list <c_object_point> points;
    c_object_transformation transformation;
};

class c_observed_scene {
public:
    c_observed_scene(c_point_cloud& point_cloud, c_world_time& world_time, string& img_path);

    // move constructors
    c_observed_scene(c_observed_scene&& scene) {
        time = scene.time;
        relations = scene.relations;
        point_cloud.points = std::move(scene.point_cloud.points);

        img_path = scene.img_path;

        pcl_octree.add_points(point_cloud, octree_ind_to_uv);
    }

    bool compatible(c_object_point& pnt, bool mark_compatible_point=true);
    void get_near_pnts(c_object_point& pnt, std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance);

    bool calculate_scene_relations();

    c_world_time time;
    c_point_cloud point_cloud;
    list <c_object_relation> relations;

    c_pcl_octree pcl_octree;
    std::vector<Vector2i> octree_ind_to_uv;

    string img_path;
};

class c_world {
public:
    
    void add_observation( c_point_cloud& point_cloud, double time, string& img_path);
    void match_observed_scene(c_observed_scene& scene);
    void match_observed_scene_relation_to_existing_objects(c_observed_scene& scene, c_object_relation& relation);
    void match_observed_scene_relation_to_previous_scenes(c_observed_scene& scene, c_object_relation& relation);
    void detect_object_in_scenes_from_transformation(c_observed_scene& scene, c_observed_scene& prev_scene, c_world_object& detected_object);

    void predict();

    std::vector < c_world_object > world_objects;
    std::vector < c_observed_scene > observed_scenes;
};

}

#endif
