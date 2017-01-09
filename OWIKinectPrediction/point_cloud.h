#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <list>
#include <set>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace ais {

typedef struct _c_point_cloud_point
{
    Vector3f X;
    Vector3f Clr;
} 	c_point_cloud_point;

class c_point_cloud {
public:
    std::vector< std::vector<c_point_cloud_point>> points;
};

class c_point_cloud_boundary
{
    public:
    std::list <c_point_cloud_point> elements;
};

}

#endif
