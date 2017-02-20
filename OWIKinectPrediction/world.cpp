#include "world.h"
#include "edge_feature.h"
#include "plane_feature.h"
#include "kinect_image.h"

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

void c_world::add_observation( c_point_cloud& point_cloud, double time) {
    c_world_time world_time(time);

    if (world_parts.empty()) {
        world_parts.push_back(new c_world_part(point_cloud, world_time));
        return;
    }
}

void c_world::predict() {
   
}

}