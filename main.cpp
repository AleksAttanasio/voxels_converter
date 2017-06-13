#include <iostream>
#include "VoxelsConversion.h"

int main() {

    VoxelsConversion bc;
    Clusters segmented_pcl;

//    segmented_pcl = bc.RegionGrowingSegment("obj_0.pcd", 5.0, 1.5, 300);

//    bc.TestFunction();
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    pcl::io::loadPCDFile("obj_0.pcd", *cloud);

    PointXYZ mass_center = bc.FindCenterOfMass(*cloud);
    PointCloud<PointXYZ>::Ptr cloud_mass;

    cloud_mass->points.push_back(mass_center);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);

    viewer = bc.SingleCloudVisualizer(cloud);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}