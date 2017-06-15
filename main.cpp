#include <iostream>
#include "VoxelsConversion.h"

int main() {

    VoxelsConversion vc;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr translated_cloud;
    boost::shared_ptr<visualization::PCLVisualizer> viewer;
    string name = "rg_cluster_0.pcd";

    io::loadPCDFile(name, *cloud);

    FloatMatrixCOM mass_center = vc.getCenterOfMass(*cloud);
    cout << mass_center << endl;

    cout << mass_center(0,0) << ", " << mass_center(1,0) << ", " << mass_center(2,0) << endl;

    translated_cloud = vc.translatePointCloud(cloud, mass_center(0,0), mass_center(1,0), mass_center(2,0));

    viewer = vc.CloudVisualizer(cloud, translated_cloud);

//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }


    return 0;
}