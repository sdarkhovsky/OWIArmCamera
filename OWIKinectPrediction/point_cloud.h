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
        Vector3f Vector;
        Vector3f Conv_X;
        Vector3f Conv_Clr;
        float gradient_mag;
        float gradient_mag_temp;
        float gradient_dir;
        float Clr_edge;
        float Edge_Curvature;
        float High_Curvature;
    } 	c_point_cloud_point;

    class c_point_cloud {
    public:
        c_point_cloud() {
        }

        void filter_by_z(float max_z);
        bool fix();

        std::vector< std::vector<c_point_cloud_point>> points;
    };

    class c_point_cloud_boundary
    {
        public:
        std::list <c_point_cloud_point> elements;
    };

    void generate_point_cloud_plane(c_point_cloud& point_cloud);
    void generate_point_cloud_prism(c_point_cloud& point_cloud);

}

#endif
