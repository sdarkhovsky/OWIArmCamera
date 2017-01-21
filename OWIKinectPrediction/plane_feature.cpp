#include <Eigen/Eigenvalues>
#include "point_cloud.h"
#include "plane_feature.h"

using namespace Eigen;

namespace ais {

    /*  sample code:
        vector < vector <bool>> plane_regions_bitmap;
        bool result = detect_plane_features(point_cloud, plane_regions_bitmap);

        std::string file_path = "C:\\Projects\\OWIArmCamera\\KinectImages\\detected_planes_point_cloud.xyz";

        c_point_cloud detected_planes_point_cloud = point_cloud;
        size_t u, v;
        for (u = 0; u < detected_planes_point_cloud.points.size(); u++) {
            for (v = 0; v < detected_planes_point_cloud.points[u].size(); v++) {
                if (plane_regions_bitmap[u][v]) {
                    detected_planes_point_cloud.points[u][v].Clr = Vector3f(250, 0, 0);
                }
            }
        }

        c_kinect_image::write_file(file_path, detected_planes_point_cloud, c_image_format::xyz);
    */
    bool detect_plane_features(c_point_cloud& point_cloud, vector < vector <bool>>& plane_regions_bitmap) {
        size_t u, v;
        size_t num_point_cloud_rows = point_cloud.points.size();
        if (num_point_cloud_rows <= 0)
            return false;
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        vector < vector <Vector3f>> normals;
        normals.resize(num_point_cloud_rows - 1);

        // calculate normals
        for (u = 0; u < num_point_cloud_rows - 1; u++) {
            if (point_cloud.points[u].size() != num_point_cloud_cols)
                return false;

            normals[u].resize(num_point_cloud_cols - 1);
            for (v = 0; v < num_point_cloud_cols - 1; v++) {
                Vector3f v1 = point_cloud.points[u][v].X - point_cloud.points[u + 1][v].X;
                Vector3f v2 = point_cloud.points[u][v + 1].X - point_cloud.points[u][v].X;
                normals[u][v] = v1.cross(v2);
                normals[u][v].normalize();
            }
        }

        // mark plane regions
        float normal_deviation_tolerance = 1.0f - 0.1f;
        plane_regions_bitmap.clear();
        plane_regions_bitmap.resize(num_point_cloud_rows);
        for (u = 0; u < num_point_cloud_rows; u++) {
            plane_regions_bitmap[u].resize(num_point_cloud_cols);
        }

        for (u = 1; u < num_point_cloud_rows - 2; u++) {
            for (v = 1; v < num_point_cloud_cols - 2; v++) {
                int num_codirected_normals = 0;
                for (size_t u1 = u - 1; u1 <= u + 1; u1++) {
                    for (size_t v1 = v - 1; v1 <= v + 1; v1++) {
                        if (u1 == u && v1 == v)
                            continue;
                        if (normals[u1][v1].dot(normals[u][v]) > normal_deviation_tolerance) {
                            num_codirected_normals++;
                        }
                    }
                }
                plane_regions_bitmap[u][v] = (num_codirected_normals > 6);
            }
        }

        return true;
    }
}
