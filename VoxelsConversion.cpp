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
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first"); // Set size of the point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "first"); // Set color of the point cloud
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
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first"); // Set size of the first point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "second"); // Set size of the second point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "first"); // Set color of the second  point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "second"); // Set color of the first point cloud
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

PointCloud<PointXYZ>::Ptr VoxelsConversion::generateCube(float x, float y, float z){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->height=1;
    int cont=0;
    for(float i=-x/2; i<x/2; i+=0.25){
        for(float j=-y/2; j<y/2; j+=0.25){
            for(float k=-z/2; k<z/2; k+=0.25){
                cont++;
                cloud->width=cont;
                cloud->points.resize(cont);
                cloud->points[cont-1].x=side_matrix_/2+i;
                cloud->points[cont-1].y=side_matrix_/2+j;
                cloud->points[cont-1].z=side_matrix_/2+k;
            }
        }
    }
    return cloud;
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

PointCloud<PointXYZ>::Ptr VoxelsConversion::translatePointCloud(PointCloud<PointXYZ>::Ptr cloud, FloatMatrixCOM trans_mat){
    Eigen::Affine3f trans=Eigen::Affine3f::Identity();
    trans.translation() << -trans_mat(0,0)+0.2, -trans_mat(1,0)+0.2, -trans_mat(2,0)+0.2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud, *translated_cloud, trans);
    return translated_cloud;
}

IntMatrix VoxelsConversion::getMatrix(PointCloud<PointXYZ>::Ptr cloud){

    IntMatrix mat;
    side_matrix_ = 30;
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
            mat[cloud->points[i].x*20][cloud->points[i].y*20][cloud->points[i].z*20]=1;
    }
    return mat;
}

void VoxelsConversion::writeMat(IntMatrix mat, const string file_name){

    std::ofstream file(file_name.c_str());
    for(int i=0; i<mat.size(); i++){
        for(int j=0; j<mat[0].size(); j++){
            for(int k=0; k<mat[0][0].size(); k++){
                file<<mat[i][j][k]<<" ";
            }
        }
    }
    file.close();
}

