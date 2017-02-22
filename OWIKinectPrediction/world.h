#ifndef WORLD_H
#define WORLD_H

#include "point_cloud.h"

#include <vector>
#include <string>
#include <memory>

namespace ais {
class c_world_time {
public:    
    c_world_time() {
        time = 0;
    };
    c_world_time(double time);
    
    double time;
};

class c_world_part {
public:
    c_world_part() {
    }
    c_world_part(c_point_cloud& point_cloud, c_world_time& world_time);

    // move constructors
    c_world_part(c_world_part&& wp) {
        time = wp.time;
        point_cloud.points = std::move(wp.point_cloud.points);
    }

    c_world_time time;
    c_point_cloud point_cloud;
};

class c_rigid_body_transformation {
public:
    Matrix3f rotation;
    Vector3f translation;
};

class c_world {
public:
    
    ~c_world() {
/*
        for (auto it = world_parts.begin(); it != world_parts.end(); it++) {
            delete *it;
        }
        world_parts.clear();
*/
    }
    void add_observation( c_point_cloud& point_cloud, double time, string& img_path);
    void match_world_part(c_world_part& wp, string& img_path);
    bool calculate_world_part_transformation(c_world_part& wp, c_world_part& prev_wp, c_point_correspondence& corner_correspondence, c_rigid_body_transformation& transformation, int switch_dir_mapping);
    bool estimate_part_points_from_transformation(c_world_part& wp, c_world_part& prev_wp, c_rigid_body_transformation& rbt, c_world_part& out_wp);
    bool compare_with_world_part(Vector3f X, Vector3f Clr, c_world_part& wp);

    void predict();

    
    std::vector < c_world_part > world_parts;
};

}

#endif
