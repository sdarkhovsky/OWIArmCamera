#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <memory>



using namespace Eigen;

namespace ais {

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
                    for (ut = -1; ut <= 1; ut++) {
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

                    Vector2f dir1_uv = Vector2f(v1, u1) - Vector2f(v, u);
                    Vector2f dir2_uv = Vector2f(v2, u2) - Vector2f(v, u);
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
}