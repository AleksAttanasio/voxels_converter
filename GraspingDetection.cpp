//

#include "GraspingDetection.h"

//
// Created by aleks on 27/02/17.

Eigen::Vector3f MathematicsOperators::PointToPointVector(PointXYZ A, PointXYZ B){

    Eigen::Vector3f distance;

    // X product
    distance(0) = (A.x - B.x);
    distance(1) = (A.y - B.y);
    distance(2) = (A.z - B.z);

    return distance;
}

Eigen::Vector3f MathematicsOperators::Dot(Eigen::Vector3f A, Eigen::Vector3f B){
    Eigen::Vector3f dot_product;

    dot_product(0) = (A(1) * B(2)) - (B(1) * A(2));
    dot_product(1) = -((A(0) * B(2)) - (B(0) * A(2)));
    dot_product(2) = (A(0) * B(1)) - (B(0) * A(1));

    return dot_product;
}

Eigen::Vector3f MathematicsOperators::VectorAdd(Eigen::Vector3f A, Eigen::Vector3f B){

    Eigen::Vector3f result;
    result(0) = A(0) + B(0);
    result(1) = A(1) + B(1);
    result(2) = A(2) + B(2);

    return result;
}

Eigen::Vector3f MathematicsOperators::VectorSub(Eigen::Vector3f A, Eigen::Vector3f B){

    Eigen::Vector3f result;
    result(0) = A(0) - B(0);
    result(1) = A(1) - B(1);
    result(2) = A(2) - B(2);

    return result;
}

float MathematicsOperators::VectorProduct(Eigen::Vector3f A, Eigen::Vector3f B){

    float result = A(0) * B(0) + (A[1] * B[1]) + (A[2] * B[2]);
    return result;
}

Eigen::Vector3f MathematicsOperators::VectorMultiplyByScalar(Eigen::Vector3f A, float k){
    Eigen::Vector3f result;

    result(0) = A(0) * k;
    result(1) = A(1) * k;
    result(2) = A(2) * k;

    return result;
}

Eigen::Vector3f MathematicsOperators::PointToVector(PointXYZ A){
    Eigen::Vector3f conv;

    conv(0) = A.x;
    conv(1) = A.y;
    conv(2) = A.z;

    return conv;
}

PointXYZ MathematicsOperators::VectorToPoint(Eigen::Vector3f A){
    PointXYZ conv;

    conv.x = A(0);
    conv.y = A(1);
    conv.z = A(2);

    return conv;
}

bool MathematicsOperators::LinePlaneIntersection (PointXYZ line_p1, PointXYZ line_p2, PointXYZ plane_p1, PointXYZ plane_p2, PointXYZ plane_p3, PointXYZ &intersection){

    float epsilon = 0.000001;
    Eigen::Vector3f first_vector = PointToPointVector(plane_p1,plane_p2);
    Eigen::Vector3f second_vector = PointToPointVector(plane_p1,plane_p3);
    Eigen::Vector3f u =  PointToPointVector(line_p1, line_p2);
    Eigen::Vector3f norm = Dot(first_vector, second_vector);
    float dot = VectorProduct(norm, u);

    if(abs(dot) > epsilon){
        Eigen::Vector3f w = PointToPointVector(line_p1, plane_p1);
        float fac = -VectorProduct(norm, w)/dot;
        u = VectorMultiplyByScalar(u, fac);
        Eigen::Vector3f result = VectorAdd(PointToVector(line_p1),u);
        intersection = VectorToPoint(result);
        return true;

    }
    else
        return false;
}

float MathematicsOperators::PointToPointEuclideanDistance(PointXYZ A, PointXYZ B){
    float result = sqrt(pow((A.x - B.x),2) + pow((A.y - B.y),2) + pow((A.z - B.z),2));

    return  result;
}

bool MathematicsOperators::GraspingPointIsValid (vector<PointXYZ> inter, vector<PointXYZ> &check){

    float min_aperture = 0.05;
    float max_aperture = 0.35;

    if(PointToPointEuclideanDistance(inter[0], inter[1]) > min_aperture && PointToPointEuclideanDistance(inter[0], inter[1]) < max_aperture){
        check = inter;
        return true;
    }
    else return false;
}

boost::shared_ptr<visualization::PCLVisualizer> GraspingDetection::GraspVisualizer (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr grasp_cloud, PointCloud<Normal>::ConstPtr normals)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZ> (grasp_cloud, "grasp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "grasp cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "grasp cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (grasp_cloud, normals, 10, 0.05, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<visualization::PCLVisualizer> GraspingDetection::NormalVisualizer (PointCloud<pcl::PointXYZ>::ConstPtr cloud, PointCloud<pcl::Normal>::ConstPtr normals){
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

vector<string> GraspingDetection::ReadDatasetPaths (char *in_file){

    double i = -1.5;

    ifstream file(in_file);
    string object;
    string path;
    string num;
    string output_name;

    vector<string> pointclouds;

    while (file.good())
    {
        getline(file, object);

        if (!object.empty()) {

            cout << "Reading file for " << object << " pointclouds..." << endl;

            while (i <= 1.5) {

                num = boost::lexical_cast<std::string>(i);
                path = "clouds/" + object + "/" + "eye" + num + ".pcd";
//                cout << path << endl;
                pointclouds.push_back(path);
                i = i + 0.5;
            }
            i = -1.5;
        }
    }
    return pointclouds;
}

pcl_database GraspingDetection::ReadPCDFromPaths (vector<string> paths, int pcd_per_obj){

    vector<vector<PointCloud<PointXYZ> > > pointcloud_dataset;
    vector<PointCloud<PointXYZ> > obj_dataset;
    PointCloud<PointXYZ> cloud;
    int k = 0; // internal counter for grouping

    cout << "Reading and organizing point cloud dataset..." << endl;

    for (std::vector<string>::const_iterator i = paths.begin(); i != paths.end(); ++i){

        io::loadPCDFile(*i,cloud);

        // divide in group of "pcd_per_obj"
        if(k < pcd_per_obj){
            obj_dataset.push_back(cloud); // save cloud in object dataset
            k++; // increase counter by 1
        }
        // when a group is complete...
        else{
            pointcloud_dataset.push_back(obj_dataset); //... save it ...
            obj_dataset.clear(); // ... clear the dataset vector ...
            k = 1; // ... reset counter ...
            obj_dataset.push_back(cloud); // ... and save the last cloud into the dataset.
        }

        // if the current step is the last one then save it
        if(i == paths.end()-1){
            pointcloud_dataset.push_back(obj_dataset);
        }
    }

    return pointcloud_dataset;
}

pcl_database GraspingDetection::SegmentDatasetPCD (pcl_database pointcloud_dataset) {

    srand(time(NULL));
    pcl_database segmented_database = pointcloud_dataset;

    int name_count = 0;

    // running over the whole dataset
    for (int i = 0; i < pointcloud_dataset.size(); i++) {

        // running over the obj dataset
        for (int k = 0; k < pointcloud_dataset[i].size(); k++) {

            cout << "Segmenting pointcloud at index [" << i << "][" << k << "] with dimension: "
                 << pointcloud_dataset[i][k].size() << " points." << endl;
            PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>);
            *cloud = pointcloud_dataset[i][k];

            double r_s = 0.02; /// The smallest scale to use in the DoN filter.
            double r_l = 0.05; /// The largest scale to use in the DoN filter.
            double threshold = 0.001; /// The minimum DoN magnitude to threshold by
            double segradius = 0.050; ///segment scene into clusters with given distance tolerance using euclidean clustering

            // Create a search tree, use KDTreee for non-organized data.
            pcl::search::Search<PointXYZ>::Ptr tree;
            if (pointcloud_dataset[i][k].isOrganized()) {
                tree.reset(new pcl::search::OrganizedNeighbor<PointXYZ>());
            } else {
                tree.reset(new pcl::search::KdTree<PointXYZ>(false));
            }

            // Set the input pointcloud for the search tree
            tree->setInputCloud(cloud);

            if (r_s >= r_l) {
                cerr << "Error: Large scale must be > small scale!" << endl;
                exit(EXIT_FAILURE);
            }

            // Compute normals using both small and large scales at each point
            pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());

            // calculate normals with the small scale
            pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
            ne.setRadiusSearch(r_s);
            ne.compute(*normals_small_scale);

            // calculate normals with the large scale
            pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
            ne.setRadiusSearch(r_l);
            ne.compute(*normals_large_scale);

            // Create output cloud for DoN results
            PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
            copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);

            cout << "Calculating DoN... " << endl;
            // Create DoN operator
            pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
            don.setInputCloud(cloud);
            don.setNormalScaleLarge(normals_large_scale);
            don.setNormalScaleSmall(normals_small_scale);

            if (!don.initCompute()) {
                std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
                exit(EXIT_FAILURE);
            }

            // Compute DoN
            don.computeFeature(*doncloud);

            // Save DoN features
            pcl::PCDWriter writer;
//            writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

            // Build the condition for filtering
            pcl::ConditionOr<PointNormal>::Ptr range_cond(new pcl::ConditionOr<PointNormal>());
            range_cond->addComparison(pcl::FieldComparison<PointNormal>::ConstPtr(
                    new pcl::FieldComparison<PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)));

            // Build the filter
            pcl::PointCloud<PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<PointNormal>);
            pcl::ConditionalRemoval<PointNormal> condrem(range_cond);
            condrem.setInputCloud(doncloud);

            // Apply filter
            condrem.filter(*doncloud_filtered);
            doncloud = doncloud_filtered;

            // Save filtered output
//            std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
//            writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);

            // Filter by magnitude
//            cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

            pcl::search::KdTree<PointNormal>::Ptr segtree(new pcl::search::KdTree<PointNormal>);
            segtree->setInputCloud(doncloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointNormal> ec;

            ec.setClusterTolerance(segradius);
            ec.setMinClusterSize(250);
            ec.setMaxClusterSize(100000);
            ec.setSearchMethod(segtree);
            ec.setInputCloud(doncloud);
            ec.extract(cluster_indices);

            int j = 0;

            // Discrimination loop: all backgrounds will be removed
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++) {

                // selecting cluster and...
                pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);

                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                    cloud_cluster_don->points.push_back (doncloud->points[*pit]); }
                // ...define it.
                cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
                cloud_cluster_don->height = 1;
                cloud_cluster_don->is_dense = true;

                // converting from PointNormal to PointXYZ
                PointCloud<PointXYZ>::Ptr cloud_cluster_norm (new PointCloud<PointXYZ>);
                cloud_cluster_norm->resize(cloud_cluster_don->size());

                for (size_t c = 0; c < cloud_cluster_don->points.size(); ++c) {
                    cloud_cluster_norm->points[c].x=cloud_cluster_don->points[c].x;
                    cloud_cluster_norm->points[c].y=cloud_cluster_don->points[c].y;
                    cloud_cluster_norm->points[c].z=cloud_cluster_don->points[c].z;}

                // extracting normals
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_removal;
                ne_removal.setInputCloud(cloud_cluster_norm);

                // Create an empty kdtree representation, and pass it to the normal estimation object.
                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_norm(new pcl::search::KdTree<pcl::PointXYZ>());
                ne_removal.setSearchMethod(tree_norm);  // set search method
                ne_removal.setRadiusSearch(0.03);   // Use all neighbors in a sphere of radius 3cm
                ne_removal.compute(*cloud_normals);    // Compute the features

                float K = 0.5; // percetage of taken points from PCL
                int num_samples = cloud_normals->size() * K; // number of samples

                vector<Normal> rnd_normal; // vector containing random normals

                // filling the rnd_normal vector with random normals
                for(int count= 0; count < num_samples; count++){
                    int index = rand() % cloud_normals->size();
                    rnd_normal.push_back(cloud_normals->at(index));
                }

                //Check if a percentage of the point of the pcl are aligned
                int similar_counter = 0; // number of matches
                float simi_tol = 0.05; // tolerance on simililarity
                float correct_tol = 20; // tolerance on correct matches

                cout << "Normal check for backgrounds detection..." << endl;
                // Compare all the normals
                for(int in = 0; in < rnd_normal.size(); in++){

                    Normal current_normal = rnd_normal.at(in);
                    int q = in;

                    while(q < rnd_normal.size()){
                        Normal cmp_normal = rnd_normal.at(q);
                        if(abs(cmp_normal.normal_x - current_normal.normal_x) < simi_tol &&
                                abs(cmp_normal.normal_y - current_normal.normal_y) < simi_tol &&
                                abs(cmp_normal.normal_z - current_normal.normal_z) < simi_tol){
                            similar_counter++;
                        }

                        q++;
                    }
                }

                float perc_correct = (float(similar_counter) /((int(rnd_normal.size())*int(rnd_normal.size()))/2))*100;

                cout << "Percentage of matches: " << perc_correct << "%" <<endl;

                // check on number of matches to state if it's a background or not
                if(perc_correct < correct_tol && perc_correct > 0)
                {
//                    CODE FOR SAVING CLOUDS AS PCD
                    stringstream ss;
                    ss << "don_cluster_" << name_count << ".pcd";
                    cout << "===> Saving PointCloud as: " << ss.str() << endl;
                    writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
                    name_count++;

                    cout << " ===> Storing cluster in dataset..." << endl;
                    segmented_database[i][k] = *cloud_cluster_norm;
                }
                else{
                    if (it == cluster_indices.end() - 1){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                        cout << "No object found. Filling dataset with empty cloud." << endl;
                        segmented_database[i][k] = *empty_cloud;
                    }
                }
            }
        }
    }

    return segmented_database;
}

obj_dataset GraspingDetection::MergeAndSubsamplePointCloud (pcl_database pointcloud_dataset){

    obj_dataset filtered_dataset; // output vector
    PointCloud<PointXYZ> merged_cloud; // cloud obtained merging objects pcl

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    cout << "Merging pointcloud of Dataset containing: " <<  pointcloud_dataset.size() << " objects." << endl;

    // Merging all clouds of the same object
    for (int i = 0; i < pointcloud_dataset.size(); ++i){
        for(int k = 0; k < pointcloud_dataset[i].size(); ++k){

            PointCloud<PointXYZ> current_cloud = pointcloud_dataset[i][k];
            merged_cloud += current_cloud;
        }

        // Converting to PCLPointCloud2
        pcl::toPCLPointCloud2(merged_cloud, *cloud);
        cout << "Number of points before filtering: " << merged_cloud.size() << endl;
        merged_cloud.clear();

        // Create the filtering object
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.01f, 0.01f, 0.01f); // Leaf size for subsampling
        sor.filter (*cloud_filtered);

        // save PCL to check validity
        pcl::PCDWriter writer;
        stringstream ss;
        ss << "obj_" << i << ".pcd";
        writer.write (ss.str(), *cloud_filtered,
                      Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

        //  converting to PointCloud again and store it in the dataset
        pcl::fromPCLPointCloud2(*cloud_filtered, merged_cloud);
        cout << "Number of points after filtering: " << merged_cloud.size() << endl;
        filtered_dataset.push_back(merged_cloud);
        merged_cloud.clear();
    }

    return filtered_dataset;
}

normal_database GraspingDetection::DetectNormalsForGrasping (obj_dataset objects_dataset){

    normal_database grasping_dataset;
    float normal_th = 0.4;
    int index_c = 0;
    float x_c;
    float y_c;
    float z_c;
    float x_ref;
    float y_ref;
    float z_ref;
    double x_rap;
    double y_rap;
    double z_rap;
    long int k = 0;
    long int i = 0;
    int norm_count = 0;
    int index_neigh = 20;

    // Initializing viewer to show normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Defining clouds and Normal Estimator
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    cout << "Evaluating possible grasping points..." << endl;


    for(int i = 0; i < objects_dataset.size(); ++i){

        *cloud = objects_dataset[i];
        cout << "Number of point in the cloud: " << cloud->size() << endl;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.03);   // Use all neighbors in a sphere of radius 3cm
        ne.compute(*cloud_normals);    // Compute the features
        cout << "Number of normals in the cloud: " << cloud_normals->points.size() << std::endl;

        std::vector<int> normals_indices;

        for (i = 0; i < cloud_normals->size(); i++) {
            index_c = i;
            x_c = cloud_normals->at(index_c).normal_x;
            y_c = cloud_normals->at(index_c).normal_y;
            z_c = cloud_normals->at(index_c).normal_z;

            k = i;

            while (k < cloud_normals->size()) {

                x_ref = cloud_normals->at(k).normal_x;
                y_ref = cloud_normals->at(k).normal_y;
                z_ref = cloud_normals->at(k).normal_z;

                x_rap = x_c/x_ref;
                y_rap = y_c/y_ref;
                z_rap = z_c/z_ref;


//            cout << "Checking index " << k << "/" << cloud_normals->size() << endl;
//            cout << "C = (" << x_c << " , " << y_c << " , " << z_c << ") \t at index " << i << endl;
//            cout << "Ref = (" << x_ref << " , " << y_ref << " , " << z_ref << ") \t at index " << k << endl;
//            cout << "Differences --> Dx = " << x_diff << "\t Dy = " << y_diff << "\t Dz = " << z_diff << endl;

                if (fabs(x_rap + 1) < normal_th && fabs(y_rap +1) < normal_th && fabs(z_rap + 1) < normal_th && i != k && abs(i - k) > index_neigh) {
//                cout << "Differences --> Dx = " << x_diff << "\t Dy = " << y_diff << "\t Dz = " << z_diff << endl;
//                    cout << "Grasping point located: normal indices are (" << i << " , " << k << ")" << endl;
                    norm_count++;
                    normals_indices.push_back(i);
                    normals_indices.push_back(k);
                }

                k++;
            }
        }

        GraspingDetection gd;
        pcl::PointCloud<pcl::PointXYZ>::Ptr grasp_cloud (new pcl::PointCloud<pcl::PointXYZ>(*cloud , normals_indices));
        pcl::PointCloud<pcl::Normal>::Ptr grasp_normal_cloud (new pcl::PointCloud<pcl::Normal>(*cloud_normals, normals_indices));
        cout << "Grasping points: " << grasp_normal_cloud->size() << endl;
        viewer = gd.GraspVisualizer(cloud,grasp_cloud, grasp_normal_cloud);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    return grasping_dataset;
}

grasping_point_dataset GraspingDetection::DetectMomentOfInertiaForGrasping (obj_dataset object_dataset){

    grasping_point_dataset grasping_points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr grasp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    for(int i = 0; i < object_dataset.size(); ++i) {

        *cloud = object_dataset[i];

        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud (cloud);
        feature_extractor.compute ();

        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia (moment_of_inertia);
        feature_extractor.getEccentricity (eccentricity);
        feature_extractor.getAABB (min_point_AABB, max_point_AABB);
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "main cloud");
//    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);
//    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
        pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
        pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
//    viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

        Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

        p1 = rotational_matrix_OBB * p1 + position;
        p2 = rotational_matrix_OBB * p2 + position;
        p3 = rotational_matrix_OBB * p3 + position;
        p4 = rotational_matrix_OBB * p4 + position;
        p5 = rotational_matrix_OBB * p5 + position;
        p6 = rotational_matrix_OBB * p6 + position;
        p7 = rotational_matrix_OBB * p7 + position;
        p8 = rotational_matrix_OBB * p8 + position;

        pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
        pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
        pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
        pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
        pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
        pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
        pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
        pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

        viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
        viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
        viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
        viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
        viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
        viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
        viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
        viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
        viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
        viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
        viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
        viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

        points_dataset planes;
        points_dataset lines;
        vector<PointXYZ> point_set;
        points_dataset grasping_pointset;

        // Plane 1-2-3-4
        point_set.push_back(pt1);
        point_set.push_back(pt2);
        point_set.push_back(pt3);
        planes.push_back(point_set);
        point_set.clear();

        //Plane 1-5-8-4
        point_set.push_back(pt1);
        point_set.push_back(pt5);
        point_set.push_back(pt8);
        planes.push_back(point_set);
        point_set.clear();

        // Plane 1-5-6-2
        point_set.push_back(pt1);
        point_set.push_back(pt5);
        point_set.push_back(pt6);
        planes.push_back(point_set);
        point_set.clear();

        // Plane 2-6-7-3
        point_set.push_back(pt2);
        point_set.push_back(pt3);
        point_set.push_back(pt6);
        planes.push_back(point_set);
        point_set.clear();

        // Plane 4-3-7-8
        point_set.push_back(pt3);
        point_set.push_back(pt4);
        point_set.push_back(pt7);
        planes.push_back(point_set);
        point_set.clear();

        // Plane 5-6-7-8
        point_set.push_back(pt5);
        point_set.push_back(pt6);
        point_set.push_back(pt7);
        planes.push_back(point_set);
        point_set.clear();

        // Line x
        point_set.push_back(center);
        point_set.push_back(x_axis);
        lines.push_back(point_set);
        point_set.clear();

        // Line y
        point_set.push_back(center);
        point_set.push_back(y_axis);
        lines.push_back(point_set);
        point_set.clear();

        MathematicsOperators mo;
        vector<PointXYZ> x_inter;
        vector<PointXYZ> y_inter;

        for(int i=0; i < planes.size(); i++){

            for(int k = 0; k < lines.size(); k++){
                PointXYZ A = lines[k][0];
                PointXYZ B = lines[k][1];
                PointXYZ P = planes[i][0];
                PointXYZ Q = planes[i][1];
                PointXYZ R = planes[i][2];
                PointXYZ inter;

                if(mo.LinePlaneIntersection(A, B, P, Q, R, inter)){
                    cout << inter << endl;
                    grasp_cloud->push_back(inter);
                    point_set.push_back(inter);

                    if(k == 0){
                        x_inter.push_back(inter);
                    }
                    else{
                        y_inter.push_back(inter);
                    }
                }
                else {
                    cout << "Impossible find the intersection point." << endl;
                }
            }
        }

        cout << mo.PointToPointEuclideanDistance(x_inter[0], x_inter[1]) << endl;
        cout << mo.PointToPointEuclideanDistance(y_inter[0], y_inter[1]) << endl;

        vector<PointXYZ> x_valid;
        vector<PointXYZ> y_valid;

        vector<vector<vector<PointXYZ> > > grasping_dataset;
        vector<vector<PointXYZ> > obj_points;

        if(mo.GraspingPointIsValid(x_inter, x_valid)){
            cout << "Points: " << x_valid[0] << ";" << x_valid[1] << "met grasping conditions." << endl;
            obj_points.push_back(x_valid);
        }
        else cout << "Grasaping points on X axis don't match he grasping condition." << endl;

        if(mo.GraspingPointIsValid(y_inter, y_valid)){
            cout << "Points: " << y_valid[0] << ";" << y_valid[1] << "met grasping conditions." << endl;
            obj_points.push_back(y_valid);
        }
        else cout << "Grasaping points on Y don't match he grasping condition." << endl;

        cout << grasping_dataset.size() << endl;

        grasping_dataset.push_back(obj_points);

        cout << grasping_dataset.size() << endl;

        viewer->addPointCloud<pcl::PointXYZ>(grasp_cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "sample cloud");

        while(!viewer->wasStopped())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    return grasping_points;

}


