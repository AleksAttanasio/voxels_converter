//
// Created by osboxes on 20/07/17.
//

#include <iostream>
#include "VoxelsConversion.h"
#include "ModelReconstruction.h"
#include "GraspingDetection.h"

PointXYZ FindBBCenter(PointXYZ A, PointXYZ B, PointXYZ C, PointXYZ D){
    PointXYZ res;

    res.x = (A.x + B.x + C.x + D.x)/4;
    res.y = (A.y + B.y + C.y + D.y)/4;
    res.z = (A.z + B.z + C.z + D.z)/4;

    return res;
}

float PointToPointEuclideanDistance(PointXYZ A, PointXYZ B){
    float result = sqrt(pow((A.x - B.x),2) + pow((A.y - B.y),2) + pow((A.z - B.z),2));

    return  result;
}

int main() {

    VoxelsConversion vc;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPCDFile("cluster_amphora_0.pcd", *cloud);
    boost::shared_ptr<visualization::PCLVisualizer> viewer = vc.SingleCloudVisualizer(cloud);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "first"); // Set size of the point cloud
    viewer->addCoordinateSystem(0);
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
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
//    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
//    viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
//    viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");

    Eigen::Vector3f p1 (min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p2 (min_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p3 (max_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p4 (max_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p5 (min_point_AABB.x, max_point_AABB.y, min_point_AABB.z);
    Eigen::Vector3f p6 (min_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p7 (max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    Eigen::Vector3f p8 (max_point_AABB.x, max_point_AABB.y, min_point_AABB.z);

//    p1 = rotational_matrix_OBB * p1 + position;
//    p2 = rotational_matrix_OBB * p2 + position;
//    p3 = rotational_matrix_OBB * p3 + position;
//    p4 = rotational_matrix_OBB * p4 + position;
//    p5 = rotational_matrix_OBB * p5 + position;
//    p6 = rotational_matrix_OBB * p6 + position;
//    p7 = rotational_matrix_OBB * p7 + position;
//    p8 = rotational_matrix_OBB * p8 + position;

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

//    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
//    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
//    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
//    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
//    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
//
//    p1 = rotational_matrix_OBB * p1 + position;
//    p2 = rotational_matrix_OBB * p2 + position;
//    p3 = rotational_matrix_OBB * p3 + position;
//    p4 = rotational_matrix_OBB * p4 + position;
//    p5 = rotational_matrix_OBB * p5 + position;
//    p6 = rotational_matrix_OBB * p6 + position;
//    p7 = rotational_matrix_OBB * p7 + position;
//    p8 = rotational_matrix_OBB * p8 + position;
//
//    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
//    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
//    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
//    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
//    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
//    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
//    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
//    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
//
//    viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
//    viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
//    viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
//    viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
//    viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
//    viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
//    viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
//    viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
//    viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
//    viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
//    viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
//    viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

    cout << pt1 << endl;
    cout << pt2 << endl;
    cout << pt5 << endl;

    PointXYZ origin =  FindBBCenter(pt4,pt3,pt7,pt8);
    float radius = PointToPointEuclideanDistance(pt1, pt2)/2;

    int classification;
    cin >> classification;


    // Cube case
    if(classification == 0){


    }

    // Cylinder case
    if(classification == 1){

        pcl::ModelCoefficients cylinder_coeff;
        cylinder_coeff.values.resize (7);    // We need 7 values
        cylinder_coeff.values[0] = origin.x;
        cylinder_coeff.values[1] = origin.y;
        cylinder_coeff.values[2] = origin.z;
        cylinder_coeff.values[3] = pt1.x - pt4.x;
        cylinder_coeff.values[4] = pt1.y - pt4.y;
        cylinder_coeff.values[5] = pt1.z - pt4.z;
        cylinder_coeff.values[6] = radius;
        viewer->addCylinder(cylinder_coeff);

    }

    /// Cone case
    if(classification == 2){
        pcl::ModelCoefficients cone_coeff;
        cone_coeff.values.resize (7);    // We need 7 values
        cone_coeff.values[0] = origin.x;
        cone_coeff.values[1] = origin.y;
        cone_coeff.values[2] = origin.z;
        cone_coeff.values[3] = pt1.x - pt4.x;
        cone_coeff.values[4] = pt1.y - pt4.y;
        cone_coeff.values[5] = pt1.z - pt4.z;
        cone_coeff.values[6] = 20;
        viewer->addCone(cone_coeff);
    }

    // Sphere case
    if (classification == 3){
        pcl::ModelCoefficients sphere_coeff;
        sphere_coeff.values.resize (4);    // We need 4 values
        sphere_coeff.values[0] = mass_center.x ();
        sphere_coeff.values[1] = mass_center.y ();
        sphere_coeff.values[2] = mass_center.z ();
        sphere_coeff.values[3] = radius*2;
        viewer->addSphere (sphere_coeff);
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


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

    pcl::PointCloud<pcl::PointXYZ>::Ptr grasp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

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

    while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
};