#ifndef PLANE_FEATURE_H
#define PLANE_FEATURE_H

#include "line_feature.h"

namespace ais {

    class c_plane_feature {
    public:
        c_line_boundary boundary;
        c_vector3f normal;
    };
}
#endif