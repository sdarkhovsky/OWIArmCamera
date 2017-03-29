#include <Eigen/Dense>
#include "point_cloud.h"
#include "edge_feature.h"
#include "png_visualize.h"

#include <fstream>
#include <iostream>
#include <string>

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
        Vector2i edge_pnt, uv;
        Vector3f cur_X, prev_X, prev_prev_X;
        float cur_dist;

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

    bool get_color_edge_chains(c_point_cloud& point_cloud, list<list<c_color_edge_node>>& edge_chains) {
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
                    std::list<c_color_edge_node> edge_chain{ std::make_move_iterator(std::begin(edge_list)),
                        std::make_move_iterator(std::end(edge_list)) };

                    edge_chains.push_back(edge_chain);
                }
            }
        }

        return true;
    }

    bool mark_edge_chains(c_point_cloud& point_cloud, list<list<c_color_edge_node>>& edge_chains) {

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


    template <class chains_type>
    void filter_chain_by_length(chains_type& chains) {
        size_t max_chain_length = 0;

        for (auto chain = chains.begin(); chain != chains.end(); chain++) {
            if (chain->size() > max_chain_length)
                max_chain_length = chain->size();
        }

        float max_chain_length_filter_value = 0.99;
        auto chain = chains.begin();
        while (chain != chains.end()) {
            if (chain->size() < (float)max_chain_length*max_chain_length_filter_value) {
                chain = chains.erase(chain);
            }
            else {
                chain++;
            }
        }
    }

    bool get_edge_chains(c_point_cloud& point_cloud, vector<vector<c_edge_node>>& edge_chains) {
        Vector2i uv;
        Vector3f X_prev, X;
        list<list<c_color_edge_node>> color_edge_chains;

        get_color_edge_chains(point_cloud, color_edge_chains);

        filter_chain_by_length(color_edge_chains);
        mark_edge_chains(point_cloud, color_edge_chains);

        edge_chains.clear();
        for (auto color_chain = color_edge_chains.begin(); color_chain != color_edge_chains.end(); color_chain++) {
            vector<c_edge_node> chain;
            // remove nodes with repeating X, which is specific of Kinect sensor, repeating same X for a nbhd of pixels, because the color resolution is greater than the depth one
            auto color_node_it = color_chain->begin();
            uv = color_node_it->uv;
            X_prev = point_cloud.points[uv(0)][uv(1)].X;
            c_edge_node node(uv, X_prev);
            chain.push_back(node);
            color_node_it++;
            while (color_node_it != color_chain->end()) {
                uv = color_node_it->uv;
                X = point_cloud.points[uv(0)][uv(1)].X;
                if (X != X_prev) {
                    c_edge_node node(uv, X);
                    chain.push_back(node);
                }
                color_node_it++;
            }

            // break edge into subedges where the adjacent nodes distance varies significantly
            vector<c_edge_node> uniform_chain;
            uniform_chain.push_back(chain[0]);
            for (int node_i = 1; node_i < chain.size()-1; node_i++) {
                float distance = (chain[node_i].X - chain[node_i - 1].X).norm();
                const float distance_sensor_resolution_tolerance = 0.01;
                if (distance > distance_sensor_resolution_tolerance) {
                    edge_chains.push_back(uniform_chain);
                    uniform_chain.clear();
                }
                uniform_chain.push_back(chain[node_i]);
            }
            edge_chains.push_back(uniform_chain);
        }


        filter_chain_by_length(edge_chains);

        for (auto chain = edge_chains.begin(); chain != edge_chains.end(); chain++) {
            calculate_edge_chain_curvature(point_cloud, *chain);
        }

        return true;
    }

    bool calculate_gaussian_masks(float sigma, float steps_per_sigma, float taper_error, vector<float>& mask, vector<float>& dmask, vector<float>& ddmask) {
        int i;
        int mask_radius;
        float sum;
        float i2;
        float step = sigma / steps_per_sigma;
        float step2 = step*step;
        float sigma2 = sigma*sigma;
        float sigma4 = sigma2*sigma2;
        float normalizer = sigma*sqrt(2 * pi);
        // gaussian
        mask_radius = steps_per_sigma+1;
        // find mask sizes
        while (true) {
            float psi = mask_radius*step;
            float val = exp(-psi*psi / (2 * sigma2))/normalizer;
            if (val < taper_error)
                break;
            mask_radius++;
        }
        // derivative of gaussian
        int dmask_radius = steps_per_sigma + 1;
        while (true) {
            float psi = dmask_radius*step;
            float val = exp(-psi*psi / (2 * sigma2))*psi / sigma2 / normalizer;
            if (val < taper_error)
                break;
            dmask_radius++;
        }
        // second derivative of gaussian
        const float SQRT_3 = 1.7320508075688772935274463415059;
        int ddmask_radius = steps_per_sigma*SQRT_3 + 1;
        // find mask sizes
        while (true) {
            float psi = ddmask_radius*step;
            float val = exp(-psi*psi / (2 * sigma2))*(psi*psi - sigma2) / sigma4 / normalizer;
            if (val < taper_error)
                break;
            ddmask_radius++;
        }

        if (mask_radius < dmask_radius) mask_radius = dmask_radius;
        if (mask_radius < ddmask_radius) mask_radius = ddmask_radius;


        mask.resize(2 * mask_radius + 1);
        dmask.resize(2 * mask_radius + 1);
        ddmask.resize(2 * mask_radius + 1);

        sum = 0;
        for (i = -mask_radius; i <= mask_radius; i++) {
            i2 = i*i*step2;
            mask[i + mask_radius] = exp(-i2 / (2.0*sigma2));
            dmask[i + mask_radius] = exp(-i2 / (2.0*sigma2))*i*step / sigma2;
            ddmask[i + mask_radius] = exp(-i2 / (2.0*sigma2))*(i2 - sigma2) / sigma4;

            sum += mask[i + mask_radius];
        }
        for (i = 0; i < mask.size(); i++) {
            mask[i] /= sum;
            dmask[i] /= sum;
            ddmask[i] /= sum;
        }

        return true;
    }

    bool calculate_edge_chain_curvature(c_point_cloud& point_cloud, vector<c_edge_node>& edge_chain) {
        Vector3f X_i, X_ip, X_t;
        float curvature;
        vector<float> mask, dmask, ddmask;

#define TEST_EDGE_SMOOTHING
#ifdef TEST_EDGE_SMOOTHING
        Vector3f X_s;
#endif
        Vector3f dX_dt, d2X_dt2;

        int chain_size = edge_chain.size();

#if 1
        for (int node_i = 0; node_i < chain_size; node_i++) {
            Vector2i uv = edge_chain[node_i].uv;
            Vector3f X = point_cloud.points[uv(0)][uv(1)].X;
            Vector3f X_prev = Vector3f::Zero();
            Vector2i uv_prev;
            if (node_i > 0) {
                uv_prev = edge_chain[node_i-1].uv;
                X_prev = point_cloud.points[uv_prev(0)][uv_prev(1)].X;
            }

//            std::cout << " node_i= " << node_i << " uv= (" << uv(0) << ", " << uv(1) << ")" << " X= (" << X(0) << ", " << X(1) << ", " << X(2) << ")" << " dist= " << (X-X_prev).norm() << std::endl;
        }
        std::cout << std::endl;
#endif

        // Tried configurations (sigma, steps_per_sigma, mask_radius): (1,2, 12), (2,2, 11), (1,1,6), (3,6, 31), (2,4,22), (3,3,16), (4,4,20)
        float sigma = 3;  // in number of nodes as that's the parameter used in piecewise-linear representation X(t)
        float steps_per_sigma = 3;
        float t_step = sigma/ steps_per_sigma;
        const float gaussian_mask_taper_error = 0.000001; // should be small enough for ddmask as values are small there; 0.00001 should do too.
        if (!calculate_gaussian_masks(sigma, steps_per_sigma, gaussian_mask_taper_error, mask, dmask, ddmask))
            return false;
        int mask_radius = (mask.size()-1)/2;

        for (int node_i = 0; node_i < chain_size; node_i++) {
#ifdef TEST_EDGE_SMOOTHING
            X_s = Vector3f::Zero();
#endif
            dX_dt = Vector3f::Zero();
            d2X_dt2 = Vector3f::Zero();
            float tau = node_i;
            for (int k = -mask_radius; k <= mask_radius; k++) {
                // before smoothing with the gaussian, the X(t) function is piecewise linear.
                float t = tau + k*t_step; 
                float t_i = floor(t);
                float t_ip = t_i+1;

                if (t_i < 0) {
                    X_t = point_cloud.points[edge_chain[0].uv(0)][edge_chain[0].uv(1)].X;
                }
                else
                    if (t_ip >= chain_size) {
                        X_t = point_cloud.points[edge_chain[chain_size-1].uv(0)][edge_chain[chain_size-1].uv(1)].X;
                    }
                    else {
                        X_i = point_cloud.points[edge_chain[t_i].uv(0)][edge_chain[t_i].uv(1)].X;
                        X_ip = point_cloud.points[edge_chain[t_ip].uv(0)][edge_chain[t_ip].uv(1)].X;
                        X_t = X_i + (X_ip - X_i)*(t - t_i);
                    }

//#define TEST_SHAPES
#ifdef TEST_SHAPES
                    //X_t = t*Vector3f(1.0, 2.0, 1.5);
                    float angle_step = pi / 180.0;
                    /* 1 in the tau unit of measure means here the angle step. For example current node_i is 4, and t_step is 0.5
                       Then t changes as   ...,1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, ...
                       This will correspond to the angles ...1.5*angle_step,... 5.5*angle_step,...
                       at which X_t is evaluated and multiplied by the mask, dmask,ddmask
                       X_t corrsponds to 
                    */
                    float radius = 0.02;
                    X_t = radius* Vector3f(cos(angle_step*t), sin(angle_step*t), 0);
                    // dX/dt is is radius*angle_step*Vector3f(-sin(angle_step*t), cos(angle_step*t), 0);  
                    // the norm of dX/dt is radius*angle_step (= 0.02* pi / 180.0 = 3.4906585039886591538473815369772e-4)
                    // d2X/dt2 is is radius*angle_step*angle_step*Vector3f(-cos(angle_step*t), -sin(angle_step*t), 0); 
                    // the norm of d2X/dt2 is radius*angle_step*angle_step (= 0.02* pi / 180.0 * pi / 180.0 = 6.0923483957341719869348709875779e-6) 
                    //
                    // for the circle the change of the parameter t by 1 corresponds to the change angle_step in terms of angle (1 degree for pi/180). Then curvature is calculated correctly for the radius 0.02.
                    // for a general curve the change of the parameter by 1 corresponds to moving from a node to a node along the edge. The less than half circle on the OWI image contains about 56 samples, not exactly uniformely distributed along the edge
                    // for the angle_step pi/50 the curvature and radius are still calculated correctly
                    // Changing the value of sigma changes the shape of the curve (by smoothing) and thus changes the calculated curvatures (because the curve changed).
#endif
#ifdef TEST_EDGE_SMOOTHING
                X_s += X_t * mask[k + mask_radius];
#endif
                dX_dt += X_t * dmask[k + mask_radius];
                d2X_dt2 += X_t * ddmask[k + mask_radius];
            }
            edge_chain[node_i].curvature = dX_dt.cross(d2X_dt2).norm() / pow(dX_dt.norm(), 3.0);
#if 1
            float nom = dX_dt.cross(d2X_dt2).norm();
            float denom = pow(dX_dt.norm(), 3.0);

            //std::cout << " node_i= " << node_i << " nom= " << nom << " denom=" << denom << " curvature=" << edge_chain[node_i].curvature << " radius=" << 1.0 / edge_chain[node_i].curvature << " dX_dt.norm()= " << dX_dt.norm() << " d2X_dt2.norm()= " << d2X_dt2.norm() << std::endl;
            std::cout << " node_i= " << node_i << " uv= (" << edge_chain[node_i].uv(0) << ", " << edge_chain[node_i].uv(1) << ")" << " curvature=" << edge_chain[node_i].curvature << " radius=" << 1.0/edge_chain[node_i].curvature;
            if (node_i > 0) {
                float c0 = edge_chain[node_i-1].curvature;
                float c1 = edge_chain[node_i].curvature;
                std::cout << " curv.diff=" << abs(c1 - c0) / c0 << " radius.diff=" << abs(c0 - c1)/c1 << std::endl;
            }
            else {
                std::cout << std::endl;
            }
#endif
            edge_chain[node_i].normal = dX_dt.cross(d2X_dt2);
            edge_chain[node_i].normal.normalize();

#ifdef TEST_EDGE_SMOOTHING
            Vector2i uv = edge_chain[node_i].uv + Vector2i(-1, -1);
            point_cloud.points[uv(0)][uv(1)].X = X_s;
            point_cloud.points[uv(0)][uv(1)].Clr = Vector3f(255,0,0);
            point_cloud.points[uv(0)][uv(1)].Label = Vector3i(2, 0, 0);
#endif
        }

        std::cout << std::endl;
        return true;
    }
}