#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <memory>



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

    // edge_dir must be normalized
    bool advance_step_along_edge(const c_point_cloud& point_cloud, Vector2i& edge_dir, Vector2i& edge_pnt) {
        int ut, vt, u1, v1;
        int u = edge_pnt(0);
        int v = edge_pnt(1);
        int max_deviation = -1; // the point opposite to the edge_dir
        int deviation;
        assert(edge_dir.norm() == 1.0);
        Vector2i dir, new_edge_dir, new_edge_pnt;
        for (ut = -1; ut <= 1; ut++) {
            for (vt = -1; vt <= 1; vt++) {
                if (ut == 0 && vt == 0)
                    continue;
                u1 = u + ut;
                v1 = v + vt;
                if (point_cloud.points[u1][v1].Clr_edge != 0) {
                    dir = Vector2i(u1 - u, v1 - v);
                    dir.normalize();
                    deviation = edge_dir.dot(dir);
                    if (deviation > max_deviation) {
                        max_deviation = deviation;
                        new_edge_dir = dir;
                        new_edge_pnt = Vector2i(u1, v1);
                    }
                }
            }
        }

        if (max_deviation > -1) {
            edge_dir = new_edge_dir;
            edge_pnt = new_edge_pnt;
            return true;
        }
        return false;
    }

    bool advance_along_edge(const c_point_cloud& point_cloud, Vector2i& edge_dir, Vector2i& edge_pnt) {
        size_t u2, v2, j;
        const int max_num_advance_iter = 20;
        const float min_edge_pnt_distance = 0.01f; 
        const float max_edge_pnt_distance = 1.0f; 
        float edge_pnt_distance;
        size_t u1 = edge_pnt(0);
        size_t v1 = edge_pnt(1);

        for (j = 0; j < max_num_advance_iter; j++) {
            if (!advance_step_along_edge(point_cloud, edge_dir, edge_pnt))
                break;

            u2 = edge_pnt(0);
            v2 = edge_pnt(1);
            // Requiring the min allowed distance between the corner points increases robustness of the corner relation.
            // In addition, it also filters out repeated X coordinates for different u,v in Kinect data
            edge_pnt_distance = (point_cloud.points[u2][v2].X - point_cloud.points[u1][v1].X).norm();
            if (edge_pnt_distance > min_edge_pnt_distance &&
                edge_pnt_distance < max_edge_pnt_distance)
                return true;
        }

        return false;
    }

    bool find_edge_corners(c_point_cloud& point_cloud) {
        size_t u, v, j;
        int i1, i2, i3;
        int u1, v1;
        int u2, v2;
        Vector2i corner_pts[3]; // u,v coordinates of the corner points
        Vector3f dir1, dir2, X, X1, X2;

//        smooth_edge_X_Gaussian(point_cloud, 2.0, 6);   // other values: (2.0, 6), (1.0, 2), (1.4, 4)

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();
        int ut, vt;
        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {

#if 0
                //1111111111111111111111111111111111111111111
                bool pass = ((u == 193 && v == 771));
                if (!pass)
                    continue;
                //1111111111111111111111111111111111111111111
#endif


                if (point_cloud.points[u][v].Clr_edge) {
                    int num_edge_nghbrs = 0;
                    Vector2i edge_pnt;
                    Vector2i init_edge_dir;
                    for (ut = -1; ut <= 1;ut++) {
                        for (vt = -1; vt <= 1; vt++) {
                            if (ut == 0 && vt == 0) 
                                continue;
                            u1 = u + ut;
                            v1 = v + vt;
                            if (point_cloud.points[u1][v1].Clr_edge != 0) {
                                num_edge_nghbrs++;
                                edge_pnt = Vector2i(u1, v1);
                            }
                        }
                    }

                    if (num_edge_nghbrs < 2) {
//#define MARK_EDGE_ENDS
#ifdef MARK_EDGE_ENDS
                        if (num_edge_nghbrs == 1) {
                            point_cloud.points[u][v].edge_corner = 1.0;
                        }
#endif
                        continue;
                    }

                    init_edge_dir = edge_pnt - Vector2i(u, v);
                    init_edge_dir.normalize();
                    Vector2i edge_dir = init_edge_dir;

                    corner_pts[0] = Vector2i(u, v);
                    edge_pnt = corner_pts[0];

                    if (!advance_along_edge(point_cloud, edge_dir, edge_pnt))
                        continue;

                    corner_pts[1] = edge_pnt;

                    // advance in opposite direction
                    edge_dir = -init_edge_dir;
                    edge_pnt = Vector2i(u, v);

                    if (!advance_along_edge(point_cloud, edge_dir, edge_pnt))
                        continue;

                    corner_pts[2] = edge_pnt;

                    // by now corner_pts contain 3 consecutive points of the edge with suspected corner point in the middle (corner_pts[1])
                    X = point_cloud.points[corner_pts[0](0)][corner_pts[0](1)].X;
                    X1 = point_cloud.points[corner_pts[1](0)][corner_pts[1](1)].X;
                    X2 = point_cloud.points[corner_pts[2](0)][corner_pts[2](1)].X;
                    dir1 = X1 - X;
                    dir2 = X2 - X;
                    dir1.normalize();
                    dir2.normalize();

                    float angle_cos = dir1.dot(dir2);

                    const float corner_angle_cosine_thresh = cos(135.0 / 180.0*pi);
                    if (angle_cos > corner_angle_cosine_thresh) {
                        point_cloud.points[u][v].edge_corner_angle_cos = angle_cos;
                        point_cloud.points[u][v].edge_corner_dir1 = dir1;
                        point_cloud.points[u][v].edge_corner_dir2 = dir2;
                    }
#if 0
                    dir1 = point_cloud.points[u1][v1].X - point_cloud.points[u][v].X;
                    dir2 = point_cloud.points[u2][v2].X - point_cloud.points[u][v].X;
                    dir1.normalize();
                    dir2.normalize();

                    // on the edges there is often big difference in Z coordinate when the edge also corresponds to a step in Z direction
                    // to filter such cases of incorrect flagging straight line segments as corners, we calculate also angles between xy directions

                    Vector2f dir1_xy = Vector2f(point_cloud.points[u1][v1].X(0), point_cloud.points[u1][v1].X(1)) - Vector2f(point_cloud.points[u][v].X(0), point_cloud.points[u][v].X(1));
                    Vector2f dir2_xy = Vector2f(point_cloud.points[u2][v2].X(0), point_cloud.points[u2][v2].X(1)) - Vector2f(point_cloud.points[u][v].X(0), point_cloud.points[u][v].X(1));
                    dir1_xy.normalize();
                    dir2_xy.normalize();

                    Vector2f dir1_uv = Vector2f(v1,u1) - Vector2f(v,u);
                    Vector2f dir2_uv = Vector2f(v2, u2) - Vector2f(v,u);
                    dir1_uv.normalize();
                    dir2_uv.normalize();

                    // calculating angle between tangent rather than curvature is less prone to the noise
                    float corner_angle_cosine = dir1.dot(dir2);
                    float corner_angle_cosine_xy = dir1_xy.dot(dir2_xy);
                    float corner_angle_cosine_uv = dir1_uv.dot(dir2_uv);
                    if (corner_angle_cosine_uv > corner_angle_cosine_thresh) {
//                    if (corner_angle_cosine > corner_angle_cosine_thresh && corner_angle_cosine_xy > corner_angle_cosine_thresh) {
                        point_cloud.points[u][v].edge_corner = 1.0;
                    }
#endif
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
        png_visualize_point_cloud(png_file_path.c_str(), point_cloud);
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