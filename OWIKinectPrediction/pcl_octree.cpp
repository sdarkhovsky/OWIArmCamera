#include "pcl_octree.h"

#include <iostream>
#include <vector>
#include <ctime>

namespace ais {

    bool c_pcl_octree::add_points(c_point_cloud& point_cloud, std::vector<Vector2i>& octree_ind_to_uv) {

        size_t num_point_cloud_rows = point_cloud.points.size();
        size_t num_point_cloud_cols = point_cloud.points[0].size();

        // calculate the number of non-zero points
        size_t num_non_zero_pnts = 0;
        for (size_t u = 0; u < num_point_cloud_rows; u++) {
            for (size_t v = 0; v < num_point_cloud_cols; v++) {
                if (point_cloud.points[u][v].X == Vector3f::Zero()) // the input .kin files have no points for some u,v
                    continue;
                num_non_zero_pnts++;
            }
        }

        cloud->width = num_non_zero_pnts;
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        octree_ind_to_uv.resize(num_non_zero_pnts);

        size_t ind = 0;
        for (size_t u = 0; u < num_point_cloud_rows; u++) {
            for (size_t v = 0; v < num_point_cloud_cols; v++) {
                if (point_cloud.points[u][v].X == Vector3f::Zero()) // the input .kin files have no points for some u,v
                    continue;

                Vector3f& X = point_cloud.points[u][v].X;
                pcl::PointXYZ& pnt = cloud->points[ind];
                pnt.x = X(0);
                pnt.y = X(1);
                pnt.z = X(2);

                octree_ind_to_uv[ind] = Vector2i(u, v);
                ind++;
            }
        }

        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        return true;
    }


    // Neighbors within voxel search
    // results: cloud->points[pointIdxVec[i]]
    bool c_pcl_octree::voxelSearch(pcl::PointXYZ& searchPoint, std::vector<int>& pointIdxVec) {

        return octree.voxelSearch(searchPoint, pointIdxVec);
    }

    // K nearest neighbor search
    // results: cloud->points[pointIdxNKNSearch[i]], pointNKNSquaredDistance[i]
    bool c_pcl_octree::nearestKSearch(pcl::PointXYZ& searchPoint, int K, std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance) {

        return (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0);
    }

    // Neighbors within radius search
    // results: cloud->points[pointIdxRadiusSearch[i]], pointRadiusSquaredDistance[i]
    bool c_pcl_octree::radiusSearch(pcl::PointXYZ& searchPoint, float radius, std::vector<int>& pointIdxRadiusSearch, std::vector<float>& pointRadiusSquaredDistance) {
        return (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0);
    }
}
