#ifndef KINECT_IMAGE_H
#define KINECT_IMAGE_H

#include <vector>
#include <string>
#include "point_cloud.h"

namespace ais {

class c_kinect_image
{
public:
	c_kinect_image() {
	}
    
    static bool read_file( std::string file_path, c_point_cloud& point_cloud );
    static bool write_file(std::string file_path, c_point_cloud& point_cloud, bool xyz_format);
};


}

#endif

