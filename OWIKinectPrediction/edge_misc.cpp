#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <memory>



using namespace Eigen;

namespace ais {

    bool smooth_edge_X_Gaussian_old(c_point_cloud& point_cloud, float sigma, int mask_radius) {
        // calculate the Gaussian mask
        int i, j;
        size_t u, v;
        int u1, v1;

        vector<vector<float>> mask;
        calculate_gaussian_mask(mask, sigma, mask_radius);

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        for (u = mask_radius; u < num_point_cloud_rows - mask_radius; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = mask_radius; v < num_point_cloud_cols - mask_radius; v++) {

                if (point_cloud.points[u][v].Clr_edge == 0)
                    continue;

                Vector3f Conv_X = Vector3f::Zero();
                float total_weight = 0;
                bool undefined_neighbours = false;
                for (u1 = -mask_radius; u1 <= mask_radius; u1++) {
                    for (v1 = -mask_radius; v1 <= mask_radius; v1++) {

                        if (point_cloud.points[u + u1][v + v1].X == Vector3f::Zero()) {
                            undefined_neighbours = true;
                        }
                        else {
                            Conv_X += point_cloud.points[u + u1][v + v1].X * mask[u1 + mask_radius][v1 + mask_radius];
                            total_weight += mask[u1 + mask_radius][v1 + mask_radius];
                        }
                    }
                }

                if (undefined_neighbours) {
                    point_cloud.points[u][v].Conv_X = Conv_X / total_weight;
                }
                else {
                    point_cloud.points[u][v].Conv_X = Conv_X;
                }
            }
        }

        return true;
    }

    bool smooth_edge_X_Gaussian(c_point_cloud& point_cloud, float sigma, int mask_radius) {
        // calculate the Gaussian mask
        int i, j;
        size_t u, v;
        int u1, v1;

        vector<vector<float>> mask;
        calculate_gaussian_mask(mask, sigma, mask_radius);

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        for (u = mask_radius; u < num_point_cloud_rows - mask_radius; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = mask_radius; v < num_point_cloud_cols - mask_radius; v++) {

                if (point_cloud.points[u][v].Clr_edge == 0)
                    continue;

                Vector3f Conv_X = Vector3f::Zero();
                float total_weight = 0;
                for (u1 = -mask_radius; u1 <= mask_radius; u1++) {
                    for (v1 = -mask_radius; v1 <= mask_radius; v1++) {

                        if (point_cloud.points[u + u1][v + v1].Clr_edge == 0)
                            continue;

                        Conv_X += point_cloud.points[u + u1][v + v1].X * mask[u1 + mask_radius][v1 + mask_radius];
                        total_weight += mask[u1 + mask_radius][v1 + mask_radius];
                    }
                }

                point_cloud.points[u][v].Conv_X = Conv_X / total_weight;
            }
        }

        return true;
    }

    // the edges between objects are not preserved between POV
    bool remove_edges_between_objects(c_point_cloud& point_cloud) {
        size_t u, v, j, u1, v1;
        int ut, vt;
        // filter out the edges for which the nbhd points 3d coordinates vary significantly 
        const int nbhd_filter_size = 2;
        const float nbhd_X_diff_tolerance = 0.03;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();
        for (u = nbhd_filter_size; u < num_point_cloud_rows - nbhd_filter_size; u++) {
            for (v = nbhd_filter_size; v < num_point_cloud_cols - nbhd_filter_size; v++) {

                if (point_cloud.points[u][v].Clr_edge) {
                    float max_nbhd_X_diff = 0;
                    for (ut = -nbhd_filter_size; ut <= nbhd_filter_size; ut++) {
                        for (vt = -nbhd_filter_size; vt <= nbhd_filter_size; vt++) {
                            if (ut == 0 && vt == 0)
                                continue;
                            u1 = u + ut;
                            v1 = v + vt;
                            float X_diff = (point_cloud.points[u1][v1].X - point_cloud.points[u][v].X).norm();
                            if (X_diff > max_nbhd_X_diff)
                                max_nbhd_X_diff = X_diff;

                            if (max_nbhd_X_diff > nbhd_X_diff_tolerance) {
                                goto exit_loops;
                            }
                        }
                    }
                exit_loops:
                    if (max_nbhd_X_diff > nbhd_X_diff_tolerance)
                        point_cloud.points[u][v].Clr_edge = 0;
                }
            }
        }

        return true;
    }

    bool calculate_boundaries(c_point_cloud& point_cloud) {

        size_t u, v, j;
        int u1, v1;
        int direction;
        int boundary_color;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        int Del_plus[4][2] = { { 1,0 },{ 1,1 },{ 0,1 },{ -1,1 } };
        int Del_minus[4][2] = { { -1,0 },{ -1,-1 },{ 0,-1 },{ 1,-1 } };

        for (u = 0; u < num_point_cloud_rows - 1; u++) {
            for (v = 0; v < num_point_cloud_cols - 1; v++) {


                if (point_cloud.points[u][v].Clr_edge != 0) {

                    // region boundaries for the edge
                    if (point_cloud.points[u][v].gradient_dir >= 0) {
                        direction = (int)(point_cloud.points[u][v].gradient_dir / (pi / 3.9));
                        boundary_color = 1;
                    }
                    else {
                        direction = (int)(pi + point_cloud.points[u][v].gradient_dir / (pi / 3.9));
                        boundary_color = 2;
                    }
                    u1 = u + Del_plus[direction][0];
                    v1 = v + Del_plus[direction][1];
                    point_cloud.points[u1][v1].clr_boundary = boundary_color;
                    u1 = u + Del_minus[direction][0];
                    v1 = v + Del_minus[direction][1];
                    point_cloud.points[u1][v1].clr_boundary = 3 - boundary_color;
                }
            }
        }

        return true;
    }
}