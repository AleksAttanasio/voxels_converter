#include <iostream>
#include "VoxelsConversion.h"

int main() {

    VoxelsConversion vc;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr translated_cloud;
    boost::shared_ptr<visualization::PCLVisualizer> viewer;
    Clusters cloud_clusters;
    IntMatrix voxel_cloud;
    string cloud_name = "obj_0";
    cin >> cloud_name;

    cloud_clusters = vc.RegionGrowingSegment(cloud_name);

    io::loadPCDFile(cloud_name + ".pcd", *cloud);

    for(int i = 0; i < cloud_clusters.size(); i++) {
        *cloud = cloud_clusters[i];

        // find center of mass
        FloatMatrixCOM mass_center = vc.getCenterOfMass(*cloud);
        cout << mass_center(0,0) << ", " << mass_center(1,0) << ", " << mass_center(2,0) << endl;

        // move the cloud to the center of mass
        translated_cloud = vc.translatePointCloud(cloud, mass_center);

        voxel_cloud = vc.getMatrix(translated_cloud);
        stringstream ss;
        ss << cloud_name << "_" << i << ".txt";
        vc.writeMat(voxel_cloud, ss.str());
        viewer = vc.CloudVisualizer(cloud, translated_cloud);


        // Visualize the voxel cloud
        int cont=0;
        for(int i=0; i<voxel_cloud.size(); i++){
            for(int j=0; j<voxel_cloud[0].size(); j++){
                for(int k=0; k<voxel_cloud[0][0].size(); k++){
                    if(voxel_cloud[i][j][k]==1){
                        std::ostringstream name;
                        name<<cont+0;
                        viewer->addCube(i, i+1, j, j+1, k, k+1, 0, 0, 0, name.str());
                        cont++;
                    }
                }
            }
        }

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

        return 0;
    }



}