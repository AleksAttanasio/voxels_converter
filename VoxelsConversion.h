//
// Created by osboxes on 06/06/17.
//

#ifndef VOXELSCONVERSION_H
#define VOXELSCONVERSION_H

#include <sstream>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/don.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <vector>
#include <ctime>

using namespace std;
using namespace pcl;

typedef vector<PointCloud<PointXYZ> > Clusters;

class VoxelsConversion {

public:
    boost::shared_ptr<visualization::PCLVisualizer> CloudVisualizer (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr second_cloud);
    boost::shared_ptr<visualization::PCLVisualizer> SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud);
    Clusters RegionGrowingSegment(string name_cloud, float smooth_th = 4.0, float curv_th = 0.5, float min_cluster_size = 200);
//    void TestFunction();
    PointXYZ FindCenterOfMass(PointCloud<PointXYZ> cloud);


};


#endif //VOXELSCONVERSION_H
