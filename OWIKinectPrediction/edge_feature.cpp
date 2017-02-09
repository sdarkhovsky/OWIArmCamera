#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"

using namespace Eigen;

namespace ais {

    
    bool detect_color_edge_features(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int u1, v1;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        // apply LOG edge detection (see Computer Vision, Shapiro, Stockman) and http://www.di.ubi.pt/~agomes/cvm/teoricas/05-edgedetection.pdf
#if 0
        const int mask_radius = 2;
        float mask[2 * mask_radius + 1][2 * mask_radius + 1] = {
                             0,  0,  -1,  0,  0,
                             0, -1,  -2, -1,  0,
                            -1, -2,  16, -2, -1,
                             0, -1,  -2, -1,  0,
                             0,  0,  -1,  0,  0
        };
#else
        const int mask_radius = 1;
        float mask[2 * mask_radius + 1][2 * mask_radius + 1] = {
                         0, -1,  0, 
                        -1,  4, -1, 
                         0, -1,  0 
        };
#endif

        // convolve with the mask
        for (u = mask_radius; u < num_point_cloud_rows - mask_radius; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = mask_radius; v < num_point_cloud_cols - mask_radius; v++) {

                if (point_cloud.points[u][v].X == Vector3f::Zero())
                    continue;

                bool undefined_neighbours = false;
                Vector3f Conv_Clr = Vector3f::Zero(); 
                for (u1 = -mask_radius; u1 <= mask_radius; u1++) {
                    for (v1 = -mask_radius; v1 <= mask_radius; v1++) {

                        if (point_cloud.points[u+u1][v+v1].X == Vector3f::Zero()) {
                            undefined_neighbours = true;
                            break;
                        }

                        Conv_Clr += point_cloud.points[u + u1][v + v1].Clr * mask[u1 + mask_radius][v1 + mask_radius];
                    }

                    if (undefined_neighbours)
                        break;
                }

                if (!undefined_neighbours) {
                    point_cloud.points[u][v].Conv_Clr = Conv_Clr;
                }
            }
        }

        // find zero crossings
        // Threshold the zero - crossings to keep only those strong ones(large difference between the positive maximum and the negative minimum) 
        // and suppress the weak zero - crossings, which most likely caused by noise.
        // http://fourier.eng.hmc.edu/e161/lectures/gradient/node8.html
        const float zero_crossing_thresh = 20;
        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                point_cloud.points[u][v].Clr_edge = 0;
                
                for (j = 0; j < 3; j++) {
                    float a = point_cloud.points[u][v].Conv_Clr(j);
                    float b = point_cloud.points[u - 1][v].Conv_Clr(j);
                    float c = point_cloud.points[u][v - 1].Conv_Clr(j);
                    if (a*b < 0 && abs(a - b) > zero_crossing_thresh) {
                        point_cloud.points[u][v].Clr_edge = 1;
                        break;
                    }
                        
                    if (a*c < 0 && abs(a - c) > zero_crossing_thresh) {
                        point_cloud.points[u][v].Clr_edge = 1;
                        break;
                    }
                }
            }
        }

        return true;
    }

    bool detect_edge_features(c_point_cloud& point_cloud) {
        size_t u, v, u1, v1, j;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        // todo: define the nbhd_radius depending on the sensor and edge detection resolution
        int nbhd_radius[2] = { 0, 2 };
        int total_nbhd_radius = nbhd_radius[0] + nbhd_radius[1];

        for (u = total_nbhd_radius; u < num_point_cloud_rows - total_nbhd_radius; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = total_nbhd_radius; v < num_point_cloud_cols - total_nbhd_radius; v++) {

                Vector3f mass_center[2];
                size_t num_points[2];

                if (point_cloud.points[u][v].X == Vector3f::Zero())
                    continue;

                for (j = 0; j < 2; j++) {
                    mass_center[j] = Vector3f::Zero();
                    num_points[j] = 0;
                }

                for (u1 = u - total_nbhd_radius; u1 <= u + total_nbhd_radius; u1++) {
                    for (v1 = v - total_nbhd_radius; v1 <= v + total_nbhd_radius; v1++) {

                        if (point_cloud.points[u1][v1].X == Vector3f::Zero())
                            continue;

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
                    point_cloud.points[u][v].Vector = edge_direction;
                }
            }
        }

        return true;
    }

}