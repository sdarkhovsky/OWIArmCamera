#include "point_cloud.h"

namespace ais {

    void c_point_cloud::filter_by_z(c_point_cloud& filtered_point_cloud, float max_z) {
        size_t u, v;
        size_t num_point_cloud_rows = points.size();
        size_t num_point_cloud_cols = points[0].size();

        filtered_point_cloud.points.resize(num_point_cloud_rows);

        for (u = 0; u < num_point_cloud_rows; u++) {
            filtered_point_cloud.points[u].resize(num_point_cloud_cols);
            for (v = 0; v < num_point_cloud_cols; v++) {
                if (points[u][v].X(2) <= max_z) {
                    filtered_point_cloud.points[u][v] = points[u][v];
                }
            }
        }

    }

    void generate_point_cloud_plane(c_point_cloud& point_cloud) {
        size_t u, v;

        size_t num_point_cloud_rows = 424;
        size_t num_point_cloud_cols = 512;
        float step_u = 0.1f;
        float step_v = 0.1f;

        // plane is ax+by+cz+d=0
        float a = 1.0f, b = 1.0f, c = 1.0f, d = -5.0f;

        point_cloud.points.resize(num_point_cloud_rows);
        for (u = 0; u < num_point_cloud_rows; u++) {
            point_cloud.points[u].resize(num_point_cloud_cols);
        }

        for (u = 0; u < num_point_cloud_rows; u++) {
            for (v = 0; v < num_point_cloud_cols; v++) {
                float x = u*step_u;
                float y = v*step_v;
                float z = (-d - a*x - b*y) / c;
                point_cloud.points[u][v].X = Vector3f(x, y, z);
                point_cloud.points[u][v].Clr = Vector3f(0, 255.0, 0);
            }
        }
    }

    void generate_point_cloud_prism(c_point_cloud& point_cloud) {
        int u, v;

        size_t num_point_cloud_rows = 424;
        size_t num_point_cloud_cols = 512;
        float step_u = 0.1f;
        float step_v = 0.1f;

        float width = (num_point_cloud_cols - 1)*step_v;
        Vector4f plane[2];

        Vector3f v1(0, 1, 0);
        Vector3f v2(-width/2, 0, width/2);
        Vector3f v3 = v1.cross(v2);
        plane[0] = Vector4f(v3(0),v3(1),v3(2), 0);

        v2 = Vector3f(width / 2, 0, width / 2);
        v3 = v2.cross(v1);
        plane[1] = Vector4f(v3(0), v3(1), v3(2), -v3(0)*(-width) - v3(1)*0 - v3(2)*0);

        point_cloud.points.resize(num_point_cloud_rows);
        for (u = 0; u < num_point_cloud_rows; u++) {
            point_cloud.points[u].resize(num_point_cloud_cols);
        }

        for (u = 0; u < num_point_cloud_rows; u++) {
            for (v = 0; v < num_point_cloud_cols; v++) {
                int pl = (v < num_point_cloud_cols / 2) ? 0 : 1;
                float x = -v*step_v;
                float y = u*step_u;
                float z = (-plane[pl](3) - plane[pl](0)*x - plane[pl](1)*y) / plane[pl](2);
                point_cloud.points[u][v].X = Vector3f(x, y, z);
                if (u > num_point_cloud_rows * 3 / 4 || u < num_point_cloud_rows / 4 || v > num_point_cloud_cols * 3 / 4 || v < num_point_cloud_cols / 4) {
                    point_cloud.points[u][v].Clr = Vector3f(0, 0, 255.0);
                }
                else {
                    point_cloud.points[u][v].Clr = Vector3f(0, 0, 128.0);
                }
            }
        }
    }

}