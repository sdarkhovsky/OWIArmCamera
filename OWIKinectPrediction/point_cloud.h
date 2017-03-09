#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>
#include <list>
#include <set>

#include <Eigen/Dense>
#include "definitions.h"

using namespace std;
using namespace Eigen;

namespace ais {

    class c_point_cloud_point {
        public:
        c_point_cloud_point() {
            X = Vector3f::Zero();
            Clr = Vector3f::Zero();
            Vector = Vector3f::Zero();
            Conv_X = Vector3f::Zero();
            Conv_Clr = Vector3f::Zero();
            gradient_mag = 0;
            gradient_mag_temp = 0;
            gradient_dir = 0;
            Clr_edge = 0;
            edge_corner_angle_cos = -1.0;
            uv = Vector2i::Zero();
            object_assigned = 0;
        }
        Vector3f X;
        Vector3f Clr;
        Vector3f Vector;
        Vector3f Conv_X;
        Vector3f Conv_Clr;
        float gradient_mag;
        float gradient_mag_temp;
        float gradient_dir;
        float Clr_edge;
        float edge_corner_angle_cos;
        Vector3f edge_corner_dir1;
        Vector3f edge_corner_dir2;
        Vector2i uv;
        int object_assigned;
    };

    class c_point_correspondence {
    public:
        c_point_correspondence(Vector2i _src_uv, Vector2i _tgt_uv) {
            src_uv = _src_uv;
            tgt_uv = _tgt_uv;
        }
        Vector2i src_uv;
        Vector2i tgt_uv;
    };

    class c_point_cloud {
        public:
        c_point_cloud() {
        }

        size_t filter_by_xyz(const Vector3f& min_xyz, const Vector3f& max_xyz);
        bool fix_uv_X_map();

        std::vector< std::vector<c_point_cloud_point>> points;
    };

    class c_point_cloud_boundary
    {
        public:
        std::list <c_point_cloud_point> elements;
    };

    void generate_point_cloud_plane(c_point_cloud& point_cloud);
    void generate_point_cloud_prism(c_point_cloud& point_cloud);

    void get_cloud_point_rendering_color(c_point_cloud_point& cloud_point, Vector3f& Clr);
}

#endif
