#include <iostream>
#include "VoxelsConversion.h"

int main() {

    VoxelsConversion vc;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr translated_cloud;
    boost::shared_ptr<visualization::PCLVisualizer> viewer;
    IntMatrix voxel_cloud;
    string name = "rg_cluster_0";

    io::loadPCDFile(name + ".pcd", *cloud);

    FloatMatrixCOM mass_center = vc.getCenterOfMass(*cloud);
    cout << mass_center << endl;

    cout << mass_center(0,0) << ", " << mass_center(1,0) << ", " << mass_center(2,0) << endl;

    translated_cloud = vc.translatePointCloud(cloud, mass_center);
    voxel_cloud = vc.getMatrix(translated_cloud);
    vc.writeMat(voxel_cloud, name + ".txt");

    viewer = vc.CloudVisualizer(cloud, translated_cloud);
//    int cont=0;
//    for(int i=0; i<voxel_cloud.size(); i++){
//        for(int j=0; j<voxel_cloud[0].size(); j++){
//            for(int k=0; k<voxel_cloud[0][0].size(); k++){
//                if(voxel_cloud[i][j][k]==1){
//                    std::ostringstream name;
//                    name<<cont+0;
//                    viewer->addCube(i, i+1, j, j+1, k, k+1, 0, 0, 0, name.str());
//                    cont++;
//                }
//            }
//        }
//    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}