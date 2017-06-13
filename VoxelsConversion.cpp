//
// Created by osboxes on 06/06/17.
//

#include "VoxelsConversion.h"

Clusters BoxelsConversion::RegionGrowingSegment(string name_cloud, float smooth_th, float curv_th, float min_cluster_size){

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
