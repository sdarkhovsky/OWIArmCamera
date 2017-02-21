#ifndef PNG_VISUALIZE_H
#define PNG_VISUALIZE_H

#include "point_cloud.h"

namespace ais {

    bool png_visualize_point_cloud(const char* file_name, ais::c_point_cloud& point_cloud);
    bool png_visualize_point_cloud_correspondence(const char* file_name, vector <c_point_correspondence>& map, c_point_cloud& src_point_cloud, c_point_cloud& tgt_point_cloud);
}

#endif