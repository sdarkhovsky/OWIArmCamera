#include "world.h"
#include "edge_feature.h"
#include "plane_feature.h"
#include "kinect_image.h"

bool get_executable_file_path(char* file_path, int file_path_length);

namespace ais {
    
c_world_time::c_world_time(double _time) {
    time = _time;
}
    
c_world_part_state::c_world_part_state(c_point_cloud& _point_cloud, c_world_time& world_time) {
    point_cloud = _point_cloud;
    time = world_time;
}

c_world_part::c_world_part(c_point_cloud& point_cloud, c_world_time& world_time) {

    bool result = detect_edge_features(point_cloud);
    
#if 1
    {
        std::string file_path = "C:\\Projects\\OWIArmCamera\\KinectImages\\detected_edges_point_cloud.xyze";

        c_point_cloud detected_edges_point_cloud = point_cloud;
        size_t u, v;
        for (u = 0; u < detected_edges_point_cloud.points.size(); u++) {
            for (v = 0; v < detected_edges_point_cloud.points[u].size(); v++) {
                 if (point_cloud.points[u][v].Edge != Vector3f::Zero()) {
                    detected_edges_point_cloud.points[u][v].Clr = Vector3f(250, 0, 0);
                }
            }
        }
        c_point_cloud filtered_point_cloud;
        detected_edges_point_cloud.filter_by_z(filtered_point_cloud, 2.0f);
        c_kinect_image::write_file(file_path, filtered_point_cloud, c_image_format::xyze);
    }
#endif
/*
    std::unique_ptr<c_world_part_state> part_state(new c_world_part_state(point_cloud, world_time));
    world_part_states.push_back(std::move(part_state));   
    */
}

void c_world::match_world_parts(std::unique_ptr<c_world_part>& world_part) {
    /*  parts of the  observed_part are matched to the parts of the world_parts 
        new world parts are created if necessary 
        new parts state for the current time are added for the existing world parts
     */
    /*
    if (world_parts.empty()) {
        world_parts.push_back(std::move(world_part));
        return;
    }
    */
}

void c_world::add_observation( c_point_cloud& point_cloud, double time) {
    c_world_time world_time(time);
    std::unique_ptr<c_world_part> observed_part(new c_world_part(point_cloud, world_time));  
    match_world_parts(observed_part);
}

void c_world::predict() {
   
}

}