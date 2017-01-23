#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"

using namespace Eigen;

namespace ais {

    bool detect_edge_features(c_point_cloud& point_cloud) {
        size_t u, v, u1, v1, j;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        int nbhd_radius[2] = { 1, 2 };
        int total_nbhd_radius = nbhd_radius[0] + nbhd_radius[1];

        for (u = total_nbhd_radius; u < num_point_cloud_rows - total_nbhd_radius; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = total_nbhd_radius; v < num_point_cloud_cols - total_nbhd_radius; v++) {

                Vector3f mass_center[2];
                size_t num_points[2];

                for (j = 0; j <= 1; j++) {
                    mass_center[j] = Vector3f::Zero();
                    num_points[j] = 0;
                }

                for (u1 = u - total_nbhd_radius; u1 <= u + total_nbhd_radius; u1++) {
                    for (v1 = v - total_nbhd_radius; v1 <= v + total_nbhd_radius; v1++) {

                        if (point_cloud.points[u1][v1].X == Vector3f::Zero()) {
                            continue;
                        }

                        if (u1 >= u - nbhd_radius[0] && u1 <= u + nbhd_radius[0] && v1 >= v - nbhd_radius[0] && v1 <= v + nbhd_radius[0]) {
                            j = 0;
                        }
                        else {
                            j = 1;
                        }

                        mass_center[j] += point_cloud.points[u1][v1].X;
                        num_points[j]++;
                    }
                }

                if (num_points[0] == 0 || num_points[1] == 0)
                    continue;

                for (j = 0; j <= 1; j++) {
                    mass_center[j] /= num_points[j];
                }

                // find edge direction
                Vector3f edge_direction = mass_center[0] - mass_center[1];
                Vector3f edge_direction_normalized = edge_direction;
                edge_direction_normalized.normalize();

                float dist_to_tangent_plane, sum_dist_to_tangent_plane = 0, tangent_plane_projection, max_tangent_plane_projection = 0;
                size_t num_pnts = 0;
                for (u1 = u - total_nbhd_radius; u1 <= u + total_nbhd_radius; u1++) {
                    for (v1 = v - total_nbhd_radius; v1 <= v + total_nbhd_radius; v1++) {

                        if (point_cloud.points[u1][v1].X == Vector3f::Zero() || (u1 == u && v1 == v)) {
                            continue;
                        }

                        Vector3f relativeX = point_cloud.points[u][v].X - point_cloud.points[u1][v1].X;
                        dist_to_tangent_plane = edge_direction_normalized.dot(relativeX);
                        Vector3f relativeX_tangent_plane_projection = relativeX - edge_direction_normalized*dist_to_tangent_plane;
                        tangent_plane_projection = relativeX_tangent_plane_projection.norm();
                        if (tangent_plane_projection > max_tangent_plane_projection)
                            max_tangent_plane_projection = tangent_plane_projection;
                        sum_dist_to_tangent_plane += dist_to_tangent_plane;
                        num_pnts++;
                    }
                }
                float ratio = sum_dist_to_tangent_plane / num_pnts;
                ratio /= max_tangent_plane_projection;
                if (ratio > 0.25f) {
                    point_cloud.points[u][v].Edge = edge_direction;
                }
            }
        }

        return true;
    }

}