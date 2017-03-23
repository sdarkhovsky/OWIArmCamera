#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <memory>
#include <map>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

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

    bool get_edge_chains_internal(c_point_cloud& point_cloud, vector<vector<c_edge_node>>& edge_chains) {
        size_t u, v, j;
        int i1, i2, i3;
        int u1, v1;
        int u2, v2;
        Vector3f dir1, dir2, X, X1, X2;

        edge_chains.clear();

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

                    // see http://stackoverflow.com/questions/5218713/one-liner-to-convert-from-listt-to-vectort
                    std::vector<c_edge_node> edge_chain{ std::make_move_iterator(std::begin(edge_list)),
                        std::make_move_iterator(std::end(edge_list)) };
                    edge_chains.push_back(edge_chain);
                }
            }
        }

        return true;
    }

    bool mark_edge_chains(c_point_cloud& point_cloud, vector<vector<c_edge_node>>& edge_chains) {

        srand(time(NULL)); // initialize random seed
        for (auto chain = edge_chains.begin(); chain != edge_chains.end(); chain++) {
            float red = rand() % 255 + 1; // generate secret number between 1 and 255
            float green = rand() % 255 + 1;
            float blue = rand() % 255 + 1;

            for (auto node = chain->begin(); node != chain->end(); node++) {
                Vector2i& uv = node->uv;
                point_cloud.points[uv(0)][uv(1)].Label = Vector3i(1, 0, 0);
                point_cloud.points[uv(0)][uv(1)].Clr = Vector3f(red, green, blue);
            }
        }

        return true;
    }

    bool get_edge_chains(c_point_cloud& point_cloud, vector<vector<c_edge_node>>& edge_chains) {
        size_t u, v;

        size_t max_chain_length = 0;
        get_edge_chains_internal(point_cloud, edge_chains);

        auto chain = edge_chains.begin();
        while (chain != edge_chains.end()) {
            if (chain->size() > max_chain_length)
                max_chain_length = chain->size();
            chain++;
        }

        chain = edge_chains.begin();
        while (chain != edge_chains.end()) {
            if (chain->size() < (float)max_chain_length*0.4) {
                chain = edge_chains.erase(chain);
            }
            else {
                chain++;
            }
        }

        for (chain = edge_chains.begin(); chain != edge_chains.end(); chain++) {
            calculate_edge_chain_curvature(point_cloud, *chain);
        }

        mark_edge_chains(point_cloud, edge_chains);

        return true;
    }

    bool calculate_gaussian_masks(float sigma_in_steps, int mask_radius_in_steps, float step, vector<float>& mask, vector<float>& dmask, vector<float>& ddmask) {
        int i;
        float i2;
        int mask_size = 2 * mask_radius_in_steps + 1;
        mask.resize(mask_size);
        float step2 = step*step;
        float sigma1 = sigma_in_steps*step;
        float sigma2 = sigma1*sigma1;
        float sigma4 = sigma2*sigma2;
        float sum = 0;
        for (i = -mask_radius_in_steps; i <= mask_radius_in_steps; i++) {
            i2 = i*i*step2;
            mask[i + mask_radius_in_steps] = exp(-i2 / (2.0*sigma2));
            dmask[i + mask_radius_in_steps] = exp(-i2 / (2.0*sigma2))* i*step/ sigma2;
            ddmask[i + mask_radius_in_steps] = exp(-i2 / (2.0*sigma2))* (i2 - sigma2)/ sigma4;

            sum += mask[i + mask_radius_in_steps];
        }
        for (i = 0; i < mask_size; i++) {
            mask[i] /= sum;
            dmask[i] /= sum;
            ddmask[i] /= sum;
        }
        return true;
    }

    bool calculate_edge_chain_curvature(c_point_cloud& point_cloud, vector<c_edge_node>& edge_chain) {
        float t_max = edge_chain.size() - 1;
        float t_step = 0.5;
        float curvature;
        vector<float> mask, dmask, ddmask;
        Vector3f X, dX_dt, d2X_dt2;

        // (2.0, 6), (1.0, 2), (1.4, 4)
        float sigma = 2.0;
        int mask_radius = 6;
        calculate_gaussian_masks(sigma, mask_radius, t_step, mask, dmask, ddmask);

        float t_margin = t_step*mask_radius;
        for (float tau = t_margin; tau < t_max- t_margin; tau += t_step) {
            dX_dt = Vector3f::Zero();
            d2X_dt2 = Vector3f::Zero();
            for (int k = -mask_radius; k <= mask_radius; k++) {
                float t = tau + k*t_step;
                float t_i = floor(t);
                Vector3f X_i = point_cloud.points[edge_chain[t_i].uv(0)][edge_chain[t_i].uv(1)].X;
                Vector3f X_ip = point_cloud.points[edge_chain[t_i+1].uv(0)][edge_chain[t_i+1].uv(1)].X;
                Vector3f X = X_i + (X_ip - X_i)*(t - t_i);
                dX_dt += X * dmask[k + mask_radius];
                d2X_dt2 += X * ddmask[k + mask_radius];
            }
            float curvature = dX_dt.cross(d2X_dt2).norm()/ dX_dt.norm();
            11111111111111111111111111111111111  review: edge_chain_curvatures corresponds to edge_chain   calculate also normal of the edge_node
            edge_chain_curvatures.push_back(curvature);
        }

        return true;
    }
}