//
// Created by osboxes on 20/07/17.
//

#include "ModelReconstruction.h"

boost::shared_ptr<visualization::PCLVisualizer> ModelReconstrction::SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ> (first_cloud, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first"); // Set size of the point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "first"); // Set color of the point cloud
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
}