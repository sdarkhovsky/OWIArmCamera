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
    Vector3f Edge;
} 	c_point_cloud_point;

class c_point_cloud {
public:
    void filter_by_z(c_point_cloud& filtered_point_cloud, float max_z);
    std::vector< std::vector<c_point_cloud_point>> points;
};

class c_point_cloud_boundary
{
    public:
    std::list <c_point_cloud_point> elements;
};

}

#endif
