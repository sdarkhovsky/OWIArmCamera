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
        out.X = rotation * (in.X + translation_before_rotation) + translation_after_rotation;
    }

    Matrix3f rotation;
    Vector3f translation_before_rotation;
    Vector3f translation_after_rotation;
    bool is_identity;
};

class c_object_relation {
public:
    c_object_relation() {};
    //    c_object_relation(c_object_relation& object_relation) = default;
    c_object_relation(const c_object_relation& object_relation) = default;
    //   c_object_relation& operator=(c_object_relation& object_relation) = default;

    virtual bool compatible(c_object_relation* relation) { return false; }
    virtual bool calculate_transformation(c_object_relation* tgt_relation, c_object_transformation& transformation, bool switch_dir_mapping) { return false; }
};

class c_corner_object_relation : public c_object_relation {
public:
    c_corner_object_relation(c_object_point& _pnt, float _angle_cos_value, Vector3f& _dir1, Vector3f& _dir2) {
        pnt = _pnt;
        angle_cos_value = _angle_cos_value;
        dir1 = _dir1;
        dir2 = _dir2;
    }

    bool compatible(c_object_relation* relation) {
        c_corner_object_relation* relation_ptr = static_cast<c_corner_object_relation*>(relation);
        return (abs(angle_cos_value - relation_ptr->angle_cos_value) < angle_cos_tolerance);
    }

    bool calculate_transformation(c_object_relation* relation, c_object_transformation& transformation, bool switch_dir_mapping) {

        c_corner_object_relation* relation_ptr = static_cast<c_corner_object_relation*>(relation);
       
        transformation.translation_before_rotation = -pnt.X;
        transformation.translation_after_rotation = relation_ptr->pnt.X;

        Vector3f src_dir1 = dir1;
        Vector3f src_dir2 = dir2;

        if (switch_dir_mapping) {
            src_dir1 = dir2;
            src_dir2 = dir1;
        }

        Matrix3f src_M, tgt_M;
        src_M << src_dir1, src_dir2, src_dir1.cross(src_dir2);
        tgt_M << relation_ptr->dir1, relation_ptr->dir2, relation_ptr->dir1.cross(relation_ptr->dir2);
        // for the rotation matrix to be truly orthogonal the angles in the realations must be the same
        transformation.rotation = tgt_M*src_M.inverse();

        transformation.is_identity = ((transformation.translation_before_rotation + transformation.translation_after_rotation).norm() < translation_zero_tolerance) &&
            ((transformation.rotation - Matrix3f::Identity()).norm() < distance_to_identity_matrix_tolerance);
        return true;
    }

    c_object_point pnt;
    float angle_cos_value;
    Vector3f dir1;
    Vector3f dir2;
};

class c_curvature_object_relation : public c_object_relation {
public:
    c_curvature_object_relation(Vector3f& start_pnt, Vector3f& end_pnt, Vector3f& normal, float curvature) {
        this->start_pnt = start_pnt;
        this->end_pnt = end_pnt;
        this->normal = normal;
        this->curvature = curvature;
    }

    bool compatible(c_object_relation* relation) {
        c_curvature_object_relation* relation_ptr = static_cast<c_curvature_object_relation*>(relation);

        return (abs(curvature - relation_ptr->curvature) < curvature_tolerance);
    }

    bool calculate_transformation(c_object_relation* relation, c_object_transformation& transformation, bool switch_dir_mapping) {

        c_curvature_object_relation* relation_ptr = static_cast<c_curvature_object_relation*>(relation);

        return false;
    }


    Vector3f start_pnt;
    Vector3f end_pnt;
    Vector3f normal;
    float curvature;
};

class c_world_object {
public:
    c_world_object(list <c_object_relation*> _relations, c_object_transformation& _transformation) : relations(_relations), transformation(_transformation) {
    }

    ~c_world_object() {
        for (auto relation_it = relations.begin(); relation_it != relations.end(); relation_it++) {
            delete *relation_it;
        }
    }

    list <c_object_relation*> relations;

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

    ~c_observed_scene() {
        for (auto relation_it = relations.begin(); relation_it != relations.end(); relation_it++) {
            delete *relation_it;
        }
    }

    bool compatible(c_object_point& pnt, bool mark_compatible_point=true);
    void get_near_pnts(c_object_point& pnt, std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance);

    bool calculate_scene_relations();
    bool calculate_edge_corner_relations();
    bool calculate_edge_curvature_relations();

    c_world_time time;
    c_point_cloud point_cloud;
    list <c_object_relation*> relations;

    c_pcl_octree pcl_octree;
    std::vector<Vector2i> octree_ind_to_uv;

    vector<vector<c_edge_node>> edge_chains;

    string img_path;
};

class c_world {
public:
    
    void add_observation( c_point_cloud& point_cloud, double time, string& img_path);
    void match_observed_scene(c_observed_scene& scene);
    void match_observed_scene_relation_to_existing_objects(c_observed_scene& scene, c_object_relation* relation);
    void match_observed_scene_relation_to_previous_scenes(c_observed_scene& scene, c_object_relation* relation);
    void detect_object_in_scenes_from_transformation(c_observed_scene& scene, c_observed_scene& prev_scene, c_world_object& detected_object);

    void predict();

    std::vector < c_world_object > world_objects;
    std::vector < c_observed_scene > observed_scenes;
};

}

#endif
