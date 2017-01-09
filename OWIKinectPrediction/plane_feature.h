#ifndef PLANE_FEATURE_H
#define PLANE_FEATURE_H

#include "line_feature.h"

namespace ais {

    bool detect_plane_features(c_point_cloud& point_cloud);

    class c_plane_feature {
    public:
        c_line_boundary boundary;
        Vector3f normal;
    };
}
#endif