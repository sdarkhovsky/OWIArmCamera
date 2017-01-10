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
}