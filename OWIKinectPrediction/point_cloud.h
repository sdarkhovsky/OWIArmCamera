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
            Clr_edge = 0;
            min_edge_corner_angle_cos = -1.0;
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
        float min_edge_corner_angle_cos;
        Vector2i uv;
    };

    class c_point_correspondence {
    public:
        c_point_correspondence(Vector2i _src_uv, float _src_angle_cos, Vector2i _tgt_uv, float _tgt_angle_cos) {
            src_uv = _src_uv;
            src_angle_cos = _src_angle_cos;
            tgt_uv = _tgt_uv;
            tgt_angle_cos = _tgt_angle_cos;
        }
        Vector2i src_uv;
        float src_angle_cos;
        Vector2i tgt_uv;
        float tgt_angle_cos;
    };

    class c_point_cloud {
        public:
        c_point_cloud() {
        }

        void filter_by_xyz(const Vector3f& min_xyz, const Vector3f& max_xyz);
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
