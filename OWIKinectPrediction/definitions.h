#ifndef DEFINITIONS_H
#define DEFINITIONS_H

namespace ais {
    // the constants used in the projects have name endings _value, _error, _tolerance
    const float pi = 3.14159265358;
    const float gaussian_mask_taper_error = 0.005;
    const float curvature_tolerance = 0.01;
    const float angle_cos_tolerance = 3.0* 2.0 / 180.0;
    const float normal_deviation_tolerance = 1.0f - 0.1f;
    const float compatible_point_color_difference_tolerance = 25.0;
    const float translation_zero_tolerance = 0.003;
    const float distance_to_identity_matrix_tolerance = 0.01;

    const float Thresh_high = 6;
    const float Thresh_low = 3;
    const float zero_crossing_thresh = 20;
}
#endif