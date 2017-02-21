#ifndef WORLD_H
#define WORLD_H

#include "point_cloud.h"

#include <vector>
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
    c_world_part(c_point_cloud& point_cloud, c_world_time& world_time);

    c_world_time time;
    c_point_cloud point_cloud;
};

class c_world {
public:
    
    ~c_world() {
        for (auto it = world_parts.begin(); it != world_parts.end(); it++) {
            delete *it;
        }
        world_parts.clear();
    }
    void add_observation( c_point_cloud& point_cloud, double time);
    void predict();

    void match_world_part(c_world_part* wp);
    
    std::string base_image_path;
    std::vector < c_world_part* > world_parts;
};

}

#endif
