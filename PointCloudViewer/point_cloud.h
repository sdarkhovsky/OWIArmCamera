#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <list>
#include <set>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace pcv {

    typedef struct _c_point_cloud_point
    {
        Vector3f X;
        Vector3f Clr;
        Vector3f Edge;
        int visible;
    } 	c_point_cloud_point;

    class c_point_cloud {
    public:
        Vector3f min_coord;
        Vector3f max_coord;

        std::vector< c_point_cloud_point> points;
        bool read_point_cloud_file(std::string file_path);
        bool write_point_cloud_file(std::string file_path);
        void reset_visibility();
    };
}

#endif
