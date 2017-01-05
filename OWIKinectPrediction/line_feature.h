#ifndef LINE_FEATURE_H
#define LINE_FEATURE_H

#include <vector>
#include "typedef.h"

using namespace std;

namespace ais {

    class c_line_feature {
    public:
        c_line_feature() {
        };
    };

    // straight line segment
    class c_line_segment_feature : public c_line_feature {
    public:
        c_line_segment_feature() {};
        c_vector3f ends[2];
    };

    class c_line_boundary {
    public:
        vector<c_line_feature> elements;
    };
}

#endif