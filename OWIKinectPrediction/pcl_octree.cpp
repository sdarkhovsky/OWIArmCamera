#include "pcl_octree.h"

#include <iostream>
#include <vector>
#include <ctime>

namespace ais {

    bool c_pcl_octree::add_points(c_point_cloud& point_cloud) {
        cloud->width = point_cloud.points[0].size();
        cloud->height = point_cloud.points.size();
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t u = 0; u < cloud->height; u++) {
            size_t row_start = u*cloud->width;
            for (size_t v = 0; v < cloud->width; v++) {
                Vector3f X = point_cloud.points[u][v].X;
                pcl::PointXYZ& pnt = cloud->points[row_start + v];
                pnt.x = X(0);
                pnt.y = X(1);
                pnt.z = X(2);
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
