#ifndef WORLD_H
#define WORLD_H

#include "point_cloud.h"

#include <vector>
#include <memory>

namespace ais {
class c_world_time {
public:    
    c_world_time(double time);
    
    double time;
}

class c_world_part_state {
public:
    c_world_part_state(c_point_cloud& point_cloud, c_world_time& world_time);
    c_time time;
    
    c_point_cloud point_cloud;
}    

class c_world_part {
public:
    c_world_part(c_point_cloud& point_cloud, c_world_time& world_time);
    
    std::vector <  std::unique_ptr<c_world_part_state> > world_part_states;    
}

class c_world {
public:
    
    void add_observation( c_point_cloud& point_cloud );
    void match_observed_parts(std::unique_ptr<c_world_part> observed_part);
    void predict();
    
    std::vector < std::unique_ptr<c_world_part> > world_parts;
}

}

#endif
