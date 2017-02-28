#ifndef DEFINITIONS_H
#define DEFINITIONS_H

namespace ais {
    const float pi = 3.14159265358;
    const float corner_angle_cosine_thresh = cos(150.0 / 180.0*pi);
    const float angle_cos_tolerance = 3.0* 2.0 / 180.0;
}
#endif