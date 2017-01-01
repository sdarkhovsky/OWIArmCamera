#include "world.h"

namespace ais {
    
c_world_time::c_world_time(double _time) {
    time = _time;
}
    
c_world_part_state::c_world_part_state(c_point_cloud& _point_cloud, c_world_time& world_time) {
    point_cloud = _point_cloud;
    time = world_time;
}

void c_world_part::c_world_part(c_point_cloud& point_cloud, c_world_time& world_time) {
    std::unique_ptr<c_world_part_state> part_state(point_cloud);
    world_part_states.push_back(std::move(part_state));   
}

void c_world::match_world_parts(std::unique_ptr<c_world_part> world_part) {
    /*  parts of the  observed_part are matched to the parts of the world_parts 
        new world parts are created if necessary 
        new parts state for the current time are added for the existing world parts
     */
    if (world_parts.empty()) {
        world_parts.push_back(std::move(world_part));
        return;
    }
    
    
}

void c_world::add_observation( c_point_cloud& point_cloud>, double time) {
    c_world_time world_time(time);
    std::unique_ptr<c_world_part> observed_part(new c_world_part(point_cloud), world_time);  
    match_world_parts(observed_parts); 
}

void c_world::predict() {
   
}

}