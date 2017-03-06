#ifndef PCL_OCTREE_H
#define PCL_OCTREE_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>  // Include struct definitions
#include <pcl/octree/octree_search.h>


#include "point_cloud.h"

namespace ais {

    const float pcl_octree_resolution = 0.001;
    class c_pcl_octree {
    public:
        c_pcl_octree() : cloud(new pcl::PointCloud<pcl::PointXYZ>), octree(pcl_octree_resolution) {};
        bool add_points(c_point_cloud& point_cloud, std::vector<Vector2i>& octree_ind_to_uv);
        bool voxelSearch(pcl::PointXYZ& searchPoint, std::vector<int>& pointIdxVec);
        bool nearestKSearch(pcl::PointXYZ& searchPoint, int K, std::vector<int>& pointIdxNKNSearch, std::vector<float>& pointNKNSquaredDistance);
        bool radiusSearch(pcl::PointXYZ& searchPoint, float radius, std::vector<int>& pointIdxRadiusSearch, std::vector<float>& pointRadiusSquaredDistance);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    };

}
#endif
