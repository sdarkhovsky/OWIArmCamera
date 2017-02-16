#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
//#include <vector>
#include <memory>

bool write_png_file(const char* file_name, ais::c_point_cloud& point_cloud);

using namespace Eigen;

namespace ais {

    bool calculate_gaussian_mask(vector<vector<float>>& mask, float sigma, int mask_radius) {
        int i, j;
        int mask_size = 2 * mask_radius + 1;
        mask.resize(mask_size);
        for (i = 0; i < mask_size; i++)
            mask[i].resize(mask_size);

        float sum = 0;
        for (i = -mask_radius; i <= mask_radius; i++) {
            for (j = -mask_radius; j <= mask_radius; j++) {
                mask[i + mask_radius][j + mask_radius] = exp(-(float)(i*i + j*j) / (2.0*sigma*sigma));
                sum += mask[i + mask_radius][j + mask_radius];
            }
        }
        for (i = 0; i < mask_size; i++) {
            for (j = 0; j < mask_size; j++) {
                mask[i][j] /= sum;
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

    bool calculate_edge_curvature(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int u1, v1;
        int u2, v2;

        int nbhrs[8][2] = { { -1,-1 },{ -1,0 },{ -1,1 },{ 0,1 },{ 1,1 },{ 1,0 },{ 1,-1 },{ 0,-1 } };

        smooth_edge_X_Gaussian(point_cloud, 1.4, 4);   // other values: (2.0, 6), (1.0, 2), (1.4, 4)

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                if (point_cloud.points[u][v].Clr_edge) {
                    int num_edge_nghbrs = 0;
                    // an edge point often has more than 2 nghbr points in the 8-nbhd
                    for (j = 0; j < 8; j++) {
                        u1 = u + nbhrs[j][0];
                        v1 = v + nbhrs[j][1];
                        if (point_cloud.points[u1][v1].Clr_edge != 0) {
                            num_edge_nghbrs++;
                            break;
                        }
                    }

                    j = (j+3) % 8; // start looking for edge continuation from almost opposite end
                    int terminate_j = (j + 7) % 8;

                    while (j != terminate_j) {
                        u2 = u + nbhrs[j][0];
                        v2 = v + nbhrs[j][1];
                        if ((u2 != u1 || v2 != v1) && point_cloud.points[u2][v2].Clr_edge != 0) {
                            num_edge_nghbrs++;
                            break;
                        }
                        j = (j+1) % 8;
                    }

                    float High_Curvature_Threshold = 10;
//#define MARK_EDGE_ENDS
#ifdef MARK_EDGE_ENDS
                    if (num_edge_nghbrs == 1) {
                        point_cloud.points[u][v].Edge_Curvature = High_Curvature_Threshold;
                        point_cloud.points[u][v].High_Curvature = point_cloud.points[u][v].Edge_Curvature;
                        continue;
                    }
#endif

                    if (num_edge_nghbrs == 2) {

                        Vector3f dX2_ds = point_cloud.points[u2][v2].X - point_cloud.points[u][v].X;
                        Vector3f dX1_ds = point_cloud.points[u][v].X - point_cloud.points[u1][v1].X;
                        float dX2_ds_norm = dX2_ds.norm();
                        float dX1_ds_norm = dX1_ds.norm();

                        if (dX2_ds_norm > 0 && dX1_ds_norm > 0) {

                            dX2_ds /= dX2_ds_norm;
                            dX1_ds /= dX1_ds_norm;
                            Vector3f d2X_ds2 = (dX2_ds - dX1_ds) / dX1_ds_norm;
                            point_cloud.points[u][v].Edge_Curvature = d2X_ds2.norm();

                            if (point_cloud.points[u][v].Edge_Curvature > High_Curvature_Threshold) {
                                point_cloud.points[u][v].High_Curvature = point_cloud.points[u][v].Edge_Curvature;
                            }
                        }
                    }
                }
            }
        }

        return true;
    }

    bool smooth_color_Gaussian(c_point_cloud& point_cloud, float sigma, int mask_radius) {
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

                if (point_cloud.points[u][v].X == Vector3f::Zero())
                    continue;

                Vector3f Conv_Clr = Vector3f::Zero();
                float total_weight = 0;
                bool undefined_neighbours = false;
                for (u1 = -mask_radius; u1 <= mask_radius; u1++) {
                    for (v1 = -mask_radius; v1 <= mask_radius; v1++) {

                        if (point_cloud.points[u + u1][v + v1].X == Vector3f::Zero()) {
                            undefined_neighbours = true;
                        }
                        else {
                            Conv_Clr += point_cloud.points[u + u1][v + v1].Clr * mask[u1 + mask_radius][v1 + mask_radius];
                            total_weight += mask[u1 + mask_radius][v1 + mask_radius];
                        }
                    }
                }

                if (undefined_neighbours) {
                    point_cloud.points[u][v].Conv_Clr = Conv_Clr / total_weight;
                }
                else {
                    point_cloud.points[u][v].Conv_Clr = Conv_Clr;
                }
            }
        }

        return true;
    }

    // see Computer Vision, Shapiro, Stockman
    bool detect_color_edge_features_Canny(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int u1, v1;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        smooth_color_Gaussian(point_cloud, 2.0, 6);   // other values: (2.0, 6), (1.0, 2), (1.4, 4)

#if 0
        for (u = 0; u < num_point_cloud_rows; u++) {
            for (v = 0; v < num_point_cloud_cols; v++) {
                point_cloud.points[u][v].Clr = point_cloud.points[u][v].Conv_Clr;
            }
        }
        std::string png_file_path = "C:\\Projects\\OWIArmCamera\\KinectImages\\smoothed.png";
        write_png_file(png_file_path.c_str(), point_cloud);
        return true;
#endif

        for (u = 0; u < num_point_cloud_rows-1; u++) {
            assert(point_cloud.points[u].size() == num_point_cloud_cols);
            for (v = 0; v < num_point_cloud_cols-1; v++) {
                // using Roberts mask
                if (point_cloud.points[u][v].Conv_Clr == Vector3f::Zero() ||
                    point_cloud.points[u][v + 1].Conv_Clr == Vector3f::Zero() ||
                    point_cloud.points[u + 1][v].Conv_Clr == Vector3f::Zero() ||
                    point_cloud.points[u + 1][v + 1].Conv_Clr == Vector3f::Zero())
                    continue;
                Vector3f color_dfdx = point_cloud.points[u][v + 1].Conv_Clr - point_cloud.points[u + 1][v].Conv_Clr;
                Vector3f color_dfdy = point_cloud.points[u][v].Conv_Clr - point_cloud.points[u + 1][v+1].Conv_Clr;
                float dfdx = 0;
                float dfdy = 0;
                float magnitude = 0;
                for (j = 0; j < 3; j++) {
                    float mag = max(abs(color_dfdx(j)), abs(color_dfdy(j)));
                    if (mag > magnitude) {
                        magnitude = mag;
                        dfdx = color_dfdx(j);
                        dfdy = color_dfdy(j);
                    }
                }

                point_cloud.points[u][v].gradient_mag = magnitude;
                point_cloud.points[u][v].gradient_dir = atan2(dfdy, dfdx);
            }
        }

        // suppress nonmaxima
        int Del_plus[4][2] = { {1,0},{1,1},{0,1},{-1,1} };
        int Del_minus[4][2] = { { -1,0 },{ -1,-1 },{ 0,-1 },{ 1,-1 } };
        float pi = 3.14159265338;
        for (u = 0; u < num_point_cloud_rows; u++) {
            for (v = 0; v < num_point_cloud_cols; v++) {
                point_cloud.points[u][v].gradient_mag_temp = point_cloud.points[u][v].gradient_mag;
            }
        }

        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                if (point_cloud.points[u][v].gradient_mag_temp == 0)
                    continue;
                int direction = (point_cloud.points[u][v].gradient_dir >= 0)
                    ? (int)(point_cloud.points[u][v].gradient_dir / (pi / 3.9))
                    : (int)(pi+point_cloud.points[u][v].gradient_dir / (pi / 3.9));
                u1 = u + Del_plus[direction][0];
                v1 = v + Del_plus[direction][1];
                if (point_cloud.points[u][v].gradient_mag_temp <= point_cloud.points[u1][v1].gradient_mag_temp)
                    point_cloud.points[u][v].gradient_mag = 0;
                u1 = u + Del_minus[direction][0];
                v1 = v + Del_minus[direction][1];
                if (point_cloud.points[u][v].gradient_mag_temp <= point_cloud.points[u1][v1].gradient_mag_temp)
                    point_cloud.points[u][v].gradient_mag = 0;
            }
        }

        // detect and follow edge
        float Thresh_high = 6;
        float Thresh_low = 3;
        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                if (point_cloud.points[u][v].Clr_edge != 0)
                    continue;

                if (point_cloud.points[u][v].gradient_mag >= Thresh_high) {
                    point_cloud.points[u][v].Clr_edge = 1.0;
                    // follow edge
                    int eu = u;
                    int ev = v;
                    while (true) {
                        for (u1 = -1; u1 <= 1; u1++) {
                            for (v1 = -1; v1 <= 1; v1++) {
                                if (u1 == 0 && v1 == 0)
                                    continue;
                                int u2 = eu + u1;
                                int v2 = ev + v1;
                                if (u2 < 1 || v2 < 1 || u2 > num_point_cloud_rows - 2 || v2 > num_point_cloud_cols - 2)
                                    continue;
                                if (point_cloud.points[u2][v2].Clr_edge != 0)
                                    continue;
                                if (point_cloud.points[u2][v2].gradient_mag > Thresh_low) {
                                    eu = u2;
                                    ev = v2;
                                    goto exit;
                                }
                            }
                        }
                        break;

                    exit:
                        point_cloud.points[eu][ev].Clr_edge = 1;
                    }
                }
            }
        }

        return true;
    }

    
    bool detect_color_edge_features_LOG(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int u1, v1;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        // apply LOG edge detection (see Computer Vision, Shapiro, Stockman) and http://www.di.ubi.pt/~agomes/cvm/teoricas/05-edgedetection.pdf
#if 0
        const int mask_radius = 2;
        const int mask_size = 2 * mask_radius + 1;
        float mask[mask_size][mask_size] = {
                             0,  0,  -1,  0,  0,
                             0, -1,  -2, -1,  0,
                            -1, -2,  16, -2, -1,
                             0, -1,  -2, -1,  0,
                             0,  0,  -1,  0,  0
        };
#else
        const int mask_radius = 1;
        const int mask_size = 2 * mask_radius + 1;
        float mask[mask_size][mask_size] = {
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


    bool detect_invalid_data_boundary(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int u1, v1;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                if (point_cloud.points[u][v].X == Vector3f::Zero())
                    continue;

                for (u1 = -1; u1 <= 1; u1++) {
                    for (v1 = -1; v1 <= 1; v1++) {
                        if (u1 == 0 && v1 == 0)
                            continue;
                        if (point_cloud.points[u + u1][v + v1].X == Vector3f::Zero()) {
                            goto boundary_point;
                        }
                    }
                }
                continue;
            boundary_point:
                point_cloud.points[u][v].Clr_edge = 1.0;
            }
        }
        return true;
    }
}