#ifndef KINECT_IMAGE_H
#define KINECT_IMAGE_H

#include <vector>
#include <string>
#include "point_cloud.h"

namespace ais {

enum class c_image_format { kinect, xyz, xyze };

class c_kinect_image
{
public:
	c_kinect_image() {
	}

    static bool read_file( std::string file_path, c_point_cloud& point_cloud );
    static bool write_file(std::string file_path, c_point_cloud& point_cloud, c_image_format image_format);
private:
    static void c_kinect_image::get_cloud_point_color(c_point_cloud_point& cloud_point, Vector3f& Clr);
};


}

#endif

