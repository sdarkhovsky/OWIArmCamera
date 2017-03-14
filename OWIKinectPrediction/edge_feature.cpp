#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <memory>



using namespace Eigen;

namespace ais {

    // edge_dir must be normalized
    bool step_along_edge(const c_point_cloud& point_cloud, Vector2i& edge_dir, Vector2i& edge_pnt) {
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
                if (point_cloud.points[u1][v1].Clr_edge != 0 && point_cloud.points[u1][v1].Clr_edge_processed == 0) {
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

    bool advance_along_edge(c_point_cloud& point_cloud, Vector2i& edge_dir, bool push_back, list<Vector2i>& edge_list) {
        Vector2i edge_pnt;

        if (push_back)
            edge_pnt = edge_list.back();
        else
            edge_pnt = edge_list.front();

        while(true) {
            if (!step_along_edge(point_cloud, edge_dir, edge_pnt))
                break;

            if (push_back)
                edge_list.push_back(edge_pnt);
            else
                edge_list.push_front(edge_pnt);

            point_cloud.points[edge_pnt(0)][edge_pnt(1)].Clr_edge_processed = 1;
        }

        return true;
    }

    bool get_edge_chains(c_point_cloud& point_cloud, list<list<Vector2i>>& edge_segments) {
        size_t u, v, j;
        int i1, i2, i3;
        int u1, v1;
        int u2, v2;
        Vector3f dir1, dir2, X, X1, X2;

        edge_segments.clear();

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();
        int ut, vt;
        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                point_cloud.points[u][v].Clr_edge_processed = 0;
            }
        }

        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {

                if (point_cloud.points[u][v].Clr_edge != 0 && point_cloud.points[u][v].Clr_edge_processed == 0) {
                    int num_edge_nghbrs = 0;
                    Vector2i edge_pnt;
                    Vector2i init_edge_dir;
                    for (ut = -1; ut <= 1; ut++) {
                        for (vt = -1; vt <= 1; vt++) {
                            if (ut == 0 && vt == 0)
                                continue;
                            u1 = u + ut;
                            v1 = v + vt;
                            if (point_cloud.points[u1][v1].Clr_edge != 0 && point_cloud.points[u][v].Clr_edge_processed == 0) {
                                num_edge_nghbrs++;
                                edge_pnt = Vector2i(u1, v1);
                            }
                        }
                    }

                    if (num_edge_nghbrs < 2) {
                        point_cloud.points[u][v].Clr_edge_processed = 0;
                        continue;
                    }

                    init_edge_dir = edge_pnt - Vector2i(u, v);
                    init_edge_dir.normalize();
                    Vector2i edge_dir = init_edge_dir;

                    edge_pnt = Vector2i(u, v);

                    list<Vector2i> edge_list;
                    edge_list.push_back(edge_pnt);
                    point_cloud.points[u][v].Clr_edge_processed = 1;

                    advance_along_edge(point_cloud, edge_dir, true, edge_list);

                    // advance in opposite direction
                    edge_dir = -init_edge_dir;
                    advance_along_edge(point_cloud, edge_dir, false, edge_list);

                    edge_segments.push_back(edge_list);
                }
            }
        }

        return true;
    }

    bool get_edge_segments(c_point_cloud& point_cloud) {
        size_t u, v;

        list<list<Vector2i>> edge_segments;

        const int min_chain_length = 20;
        get_edge_chains(point_cloud, edge_segments);

        auto segment = edge_segments.begin();
        while (segment != edge_segments.end()) {
            if (segment->size() < min_chain_length) {
                segment = edge_segments.erase(segment);
            }
            else {
                segment++;
            }
        }

        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        for (u = 1; u < num_point_cloud_rows - 1; u++) {
            for (v = 1; v < num_point_cloud_cols - 1; v++) {
                point_cloud.points[u][v].Clr_edge = 0;
            }
        }

        for (segment = edge_segments.begin(); segment != edge_segments.end(); segment++) {
            for (auto pnt = segment->begin(); pnt != segment->end(); pnt++) {
                Vector2i& uv = *pnt;
                point_cloud.points[uv(0)][uv(1)].Clr_edge = 0;
            }
        }

        return true;
    }
}