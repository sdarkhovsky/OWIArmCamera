#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>

namespace ais {

typedef struct _c_point_cloud_point
{
    float X;
    float Y;
    float Z;
    float Blue;
    float Green;
    float Red;
} 	c_point_cloud_point;

class c_point_cloud {
    public:  
    std::vector< std::vector<c_point_cloud_point>> points;
};

}

#endif
