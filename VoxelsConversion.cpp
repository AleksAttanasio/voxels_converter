//
// Created by osboxes on 06/06/17.
//

#include "VoxelsConversion.h"

boost::shared_ptr<visualization::PCLVisualizer> VoxelsConversion::SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud) {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ> (first_cloud, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "first");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<visualization::PCLVisualizer> VoxelsConversion::CloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud, PointCloud<PointXYZ>::ConstPtr second_cloud) {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.70, 0.70, 0.70);
    viewer->addPointCloud<pcl::PointXYZ> (first_cloud, "first");
    viewer->addPointCloud<pcl::PointXYZ> (second_cloud, "second");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "second");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "second");
    viewer->addCoordinateSystem(0.0);
    viewer->initCameraParameters ();
    return (viewer);
}

FloatMatrixCOM VoxelsConversion::getCenterOfMass (PointCloud<PointXYZ> cloud){

    FloatMatrixCOM center_of_mass;

    unsigned int check = pcl::compute3DCentroid(cloud,center_of_mass);

    cout << center_of_mass << endl;

    return center_of_mass;
}

Clusters VoxelsConversion::RegionGrowingSegment(string name_cloud, float smooth_th, float curv_th, float min_cluster_size){

//    Load cloud
    Clusters output_clusters;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPCDFile <PointXYZ> (name_cloud, *cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (smooth_th / 180.0 * M_PI);
    reg.setCurvatureThreshold (curv_th);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
//    std::cout << "First cluster has " << clusters[1].indices.size () << " points." << endl;
//    std::cout << "These are the indices of the points of the initial" <<
//              std::endl << "cloud that belong to the first cluster:" << std::endl;
//    int counter = 0;
//    while (counter < clusters[1].indices.size ())
//    {
//        std::cout << clusters[1].indices[counter] << ", ";
//        counter++;
//        if (counter % 10 == 0)
//            std::cout << std::endl;
//    }
//    std::cout << std::endl;

    int j = 0;
    pcl::PCDWriter writer;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it, j++)
    {
        pcl::PointCloud<PointXYZ>::Ptr cloud_cluster_don (new pcl::PointCloud<PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster_don->points.push_back (cloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;


        cout << "Saving PointCloud " << j << endl;
        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        stringstream ss;
        ss << "rg_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_don, false);
    }

//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//    }

    return output_clusters;
}

PointCloud<PointXYZ>::Ptr VoxelsConversion::rotatePointCloud(PointCloud<PointXYZ>::Ptr cloud, float rot_x, float rot_y, float rot_z){
    Eigen::Affine3f rot=Eigen::Affine3f::Identity();
    rot.rotate(Eigen::AngleAxisf(rot_x, Eigen::Vector3f::UnitX()));
    rot.rotate(Eigen::AngleAxisf(rot_y, Eigen::Vector3f::UnitY()));
    rot.rotate(Eigen::AngleAxisf(rot_z, Eigen::Vector3f::UnitZ()));
    Eigen::Affine3f trans=Eigen::Affine3f::Identity();
    trans.translation() << -side_matrix_/2, -side_matrix_/2, -side_matrix_/2;
    Eigen::Affine3f trans2=Eigen::Affine3f::Identity();
    trans2.translation() << side_matrix_/2, side_matrix_/2, side_matrix_/2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud, *rotated_cloud, trans);
    pcl::transformPointCloud(*rotated_cloud, *rotated_cloud, rot);
    pcl::transformPointCloud(*rotated_cloud, *rotated_cloud, trans2);
    return rotated_cloud;
}

PointCloud<PointXYZ>::Ptr VoxelsConversion::translatePointCloud(PointCloud<PointXYZ>::Ptr cloud, float trans_x, float trans_y, float trans_z){
    Eigen::Affine3f trans=Eigen::Affine3f::Identity();
    trans.translation() << trans_x, trans_y, trans_z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud, *translated_cloud, trans);
    return translated_cloud;
}

IntMatrix VoxelsConversion::getMatrix(PointCloud<PointXYZ>::Ptr cloud){

    IntMatrix mat;

    // Initialize matrix
    for(int l=0; l<side_matrix_; l++){
        std::vector<std::vector<int> > vec_vec;
        for(int m=0; m<side_matrix_; m++){
            std::vector<int> vec;
            for(int n=0; n<side_matrix_; n++){
                vec.push_back(0);
            }
            vec_vec.push_back(vec);
        }
        mat.push_back(vec_vec);
    }

    // Check on matrix rotation
    for(int i=0; i<cloud->size(); i++){
        if(cloud->points[i].x>=side_matrix_ || cloud->points[i].x<0 || cloud->points[i].y>=side_matrix_ || cloud->points[i].y<0 || cloud->points[i].z>=side_matrix_ || cloud->points[i].z<0){
            std::cout<<"Error rotation, out of matrix: "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
            mat.resize(0);
            return mat;
        }
        else
            mat[cloud->points[i].x][cloud->points[i].y][cloud->points[i].z]=1;
    }
    return mat;
}

//IntMatrix VoxelsConversion::getSideMatrix(IntMatrix mat){
//    IntMatrix side_mat;
//    for(int l=0; l<side_matrix_; l++){
//        std::vector<std::vector<int> > vec_vec;
//        for(int m=0; m<side_matrix_; m++){
//            std::vector<int> vec;
//            for(int n=0; n<side_matrix_; n++){
//                vec.push_back(0);
//            }
//            vec_vec.push_back(vec);
//        }
//        side_mat.push_back(vec_vec);
//    }
//
//    for(int i=0; i<mat.size(); i++){
//        for(int j=0; j<mat[0].size(); j++){
//            bool found=false;
//            for(int k=0; k<mat[0][0].size(); k++){
//                if(mat[i][j][k]==1){
//                    if(!found){
//                        side_mat[i][j][k]=1;
//                        found=true;
//                    }
//                }
//            }
//        }
//    }
//    return side_mat;
//}
//
//PointCloud<PointXYZ>::Ptr VoxelsConversion::getSidePointCloud(PointCloud<PointXYZ>::Ptr cloud_rotated, IntMatrix side_mat){
//    PointCloud<PointXYZ>::Ptr side_cloud(new PointCloud<PointXYZ>);
//    int cont=0;
//    for(int i=0; i<cloud_rotated->points.size(); i++){
//        if(side_mat[(int)cloud_rotated->points[i].x][(int)cloud_rotated->points[i].y][(int)cloud_rotated->points[i].z]==1){
//            cont++;
//            side_cloud->height=cont;
//            side_cloud->points.resize(cont);
//            side_cloud->points[cont-1].x=cloud_rotated->points[i].x;
//            side_cloud->points[cont-1].y=cloud_rotated->points[i].y;
//            side_cloud->points[cont-1].z=cloud_rotated->points[i].z;
//        }
//    }
//    return side_cloud;
//}
//
//IntMatrix4D VoxelsConversion::generateMats(PointCloud<PointXYZ>::Ptr cloud, float rand_x, float rand_y, float rand_z, float camera_rot_x, float camera_rot_y, float camera_rot_z, float camera_trans_x, float camera_trans_y, float camera_trans_z){
//
//    IntMatrix4D mats;
//    VoxelsConversion vc;
//
//    PointCloud<PointXYZ>::Ptr cloud_rotated = vc.rotatePointCloud(cloud, rand_x, rand_y, rand_z);
//
//    IntMatrix mat = vc.getMatrix(cloud_rotated);
//    if(mat.size()==0){
//        mats.resize(0);
//        return mats;
//    }
//
//    IntMatrix side_mat = vc.getSideMatrix(mat);
//
//    PointCloud<PointXYZ>::Ptr side_cloud = vc.getSidePointCloud(cloud_rotated, side_mat);
//    PointCloud<PointXYZ>::Ptr arm_cloud= vc.rotatePointCloud(cloud_rotated, camera_rot_x, camera_rot_y, camera_rot_z);
//    PointCloud<PointXYZ>::Ptr arm_side_cloud= vc.rotatePointCloud(side_cloud, camera_rot_x, camera_rot_y, camera_rot_z);
//
//    IntMatrix mat_arm = vc.getMatrix(translatePointCloud(arm_cloud, camera_trans_x, camera_trans_y, camera_trans_z));
//    if(mat_arm.size()==0){
//        mats.resize(0);
//        return mats;
//    }
//
//    IntMatrix side_mat_arm=getMatrix(translatePointCloud(arm_side_cloud, camera_trans_x, camera_trans_y, camera_trans_z));
//    if(side_mat_arm.size()==0){
//        mats.resize(0);
//        return mats;
//    }
//
//    mats.push_back(side_mat_arm);
//    mats.push_back(mat_arm);
//    return mats;
//}
//
//void VoxelsConversion::writeMat(IntMatrix mat, const std::string file_name){
//
//    std::ofstream file(file_name.c_str());
//    for(int i=0; i<mat.size(); i++){
//        for(int j=0; j<mat[0].size(); j++){
//            for(int k=0; k<mat[0][0].size(); k++){
//                file<<mat[i][j][k]<<" ";
//            }
//        }
//    }
//    file.close();
//}
//
//void VoxelsConversion::generateDataset(string cloud_name, int orientations) {
//
//    PointCloud<PointXYZ>::Ptr cloud;
//    io::loadPCDFile(cloud_name, *cloud);
//
////            float x=(((rand()%100)/100.0)*x_max_)+x_min_;
////            float y=(((rand()%100)/100.0)*y_max_)+y_min_;
////            float z=(((rand()%100)/100.0)*z_max_)+z_min_;
//
//    for (int orientation = 0; orientation < orientations; orientation++) {
//
//        bool good = false;
//        while (!good) {
//            float rand_x = (rand() % 1000) / 1000.0 * 3.1415;
//            float rand_y = (rand() % 1000) / 1000.0 * 3.1415;
//            float rand_z = (rand() % 1000) / 1000.0 * 3.1415;
//
//
//            float camera_rot_x = (rand() % 1000) / 1000.0 * 3.1415;
//            float camera_rot_y = (rand() % 1000) / 1000.0 * 3.1415;
//            float camera_rot_z = (rand() % 1000) / 1000.0 * 3.1415;
//
//            float camera_trans_x = ((rand() % 100) / 100.0 * 40) - 20;
//            float camera_trans_y = ((rand() % 100) / 100.0 * 40) - 20;
//            float camera_trans_z = ((rand() % 100) / 100.0 * 40) - 20;
//
//
//            IntMatrix4D mats = generateMats(cloud, rand_x, rand_y, rand_z, camera_rot_x, camera_rot_y, camera_rot_z,
//                                            camera_trans_x, camera_trans_y, camera_trans_z);
//            if (mats.size() != 0) {
//                good = true;
//
//                std::ostringstream id;
//                id << cont_;
//
////                writeMat(mats[0], folder_ + iterator + training_validation + "/side_objects/" + id.str() + ".txt");
////                writeMat(mats[1], folder_ + iterator + training_validation + "/complete_objects/" + id.str() + ".txt");
//            }
//
//        }
//    }
//}


