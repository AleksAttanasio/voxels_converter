//
// Created by osboxes on 20/07/17.
//

#ifndef VOXELS_CONVERTER_MODELRECONSTRUCTION_H
#define VOXELS_CONVERTER_MODELRECONSTRUCTION_H

#include <ctime>
#include <iostream>
#include <math.h>
#include <sstream>
#include <vector>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
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


class ModelReconstruction {

public:
    boost::shared_ptr<visualization::PCLVisualizer> SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud);

};


#endif //VOXELS_CONVERTER_MODELRECONSTRUCTION_H
