//
// Created by osboxes on 06/06/17.
//

#include "VoxelsConversion.h"

boost::shared_ptr<visualization::PCLVisualizer> VoxelsConversion::SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud)
{
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

boost::shared_ptr<visualization::PCLVisualizer> VoxelsConversion::CloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud, PointCloud<PointXYZ>::ConstPtr second_cloud)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.86, 0.86, 0.86);
    viewer->addPointCloud<pcl::PointXYZ> (first_cloud, "first");
    viewer->addPointCloud<pcl::PointXYZ> (second_cloud, "second");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "second");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "second");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
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

//void VoxelsConversion::TestFunction(){
//    srand ((unsigned int) time (NULL));
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Generate pointcloud data
//    cloud->width = 1000;
//    cloud->height = 1;
//    cloud->points.resize (cloud->width * cloud->height);
//
//    for (size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//        cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//        cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    }
//
//    float resolution = 128.0f;
//
//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
//
//    octree.setInputCloud (cloud);
//    octree.addPointsFromInputCloud ();
//
//    pcl::PointXYZ searchPoint;
//
//    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);
//
//    // Neighbors within voxel search
//
//    std::vector<int> pointIdxVec;
//
//    if (octree.voxelSearch (searchPoint, pointIdxVec))
//    {
//        std::cout << "Neighbors within voxel search at (" << searchPoint.x
//                  << " " << searchPoint.y
//                  << " " << searchPoint.z << ")"
//                  << std::endl;
//
//        for (size_t i = 0; i < pointIdxVec.size (); ++i)
//            std::cout << "    " << cloud->points[pointIdxVec[i]].x
//                      << " " << cloud->points[pointIdxVec[i]].y
//                      << " " << cloud->points[pointIdxVec[i]].z << std::endl;
//    }
//
//    // K nearest neighbor search
//
//    int K = 10;
//
//    std::vector<int> pointIdxNKNSearch;
//    std::vector<float> pointNKNSquaredDistance;
//
//    std::cout << "K nearest neighbor search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with K=" << K << std::endl;
//
//    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//    {
//        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
//                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
//                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
//                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
//    }
//
//    // Neighbors within radius search
//
//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;
//
//    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
//
//    std::cout << "Neighbors within radius search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with radius=" << radius << std::endl;
//
//
//    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//    {
//        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
//                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
//                      << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
//                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//    }
//}

PointXYZ VoxelsConversion::FindCenterOfMass(PointCloud<PointXYZ> cloud){

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;

    Eigen::Vector3f mass_center;
    feature_extractor.getMassCenter(mass_center);

    PointXYZ mass_center_point;

    mass_center_point.x = mass_center[0];
    mass_center_point.y = mass_center[1];
    mass_center_point.z = mass_center[2];

    return mass_center_point;

}
