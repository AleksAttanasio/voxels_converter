//
// Created by osboxes on 26/06/17.
//

#include <iostream>
#include "VoxelsConversion.h"
#include <sstream>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/don.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

using namespace pcl;
typedef vector<vector<PointXYZ> > points_dataset;
typedef vector<vector<vector<PointXYZ> > > grasping_point_dataset;

boost::shared_ptr<visualization::PCLVisualizer> SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.0, 0.0, 0.0);
    viewer->addPointCloud<pcl::PointXYZ> (first_cloud, "first");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first"); // Set size of the point cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "first"); // Set color of the point cloud
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

float PointToPointEuclideanDistance(PointXYZ A, PointXYZ B){
    float result = sqrt(pow((A.x - B.x),2) + pow((A.y - B.y),2) + pow((A.z - B.z),2));

    return  result;
}

Eigen::Vector3f PointToPointVector(PointXYZ A, PointXYZ B){

    Eigen::Vector3f distance;

    // X product
    distance(0) = (A.x - B.x);
    distance(1) = (A.y - B.y);
    distance(2) = (A.z - B.z);

    return distance;
}

bool GraspingPointIsValid (vector<PointXYZ> inter, vector<PointXYZ> &check){

    float min_aperture = 0.05;
    float max_aperture = 1.5;

    if(PointToPointEuclideanDistance(inter[0], inter[1]) > min_aperture && PointToPointEuclideanDistance(inter[0], inter[1]) < max_aperture){
        check = inter;
        return true;
    }
    else return false;
}

Eigen::Vector3f Dot(Eigen::Vector3f A, Eigen::Vector3f B){
    Eigen::Vector3f dot_product;

    dot_product(0) = (A(1) * B(2)) - (B(1) * A(2));
    dot_product(1) = -((A(0) * B(2)) - (B(0) * A(2)));
    dot_product(2) = (A(0) * B(1)) - (B(0) * A(1));

    return dot_product;
}

float VectorProduct(Eigen::Vector3f A, Eigen::Vector3f B){

    float result = A(0) * B(0) + (A[1] * B[1]) + (A[2] * B[2]);
    return result;
}

Eigen::Vector3f VectorMultiplyByScalar(Eigen::Vector3f A, float k){
    Eigen::Vector3f result;

    result(0) = A(0) * k;
    result(1) = A(1) * k;
    result(2) = A(2) * k;

    return result;
}

PointXYZ VectorToPoint(Eigen::Vector3f A){
    PointXYZ conv;

    conv.x = A(0);
    conv.y = A(1);
    conv.z = A(2);

    return conv;
}

Eigen::Vector3f PointToVector(PointXYZ A){
    Eigen::Vector3f conv;

    conv(0) = A.x;
    conv(1) = A.y;
    conv(2) = A.z;

    return conv;
}


Eigen::Vector3f VectorAdd(Eigen::Vector3f A, Eigen::Vector3f B){

    Eigen::Vector3f result;
    result(0) = A(0) + B(0);
    result(1) = A(1) + B(1);
    result(2) = A(2) + B(2);

    return result;
}


bool LinePlaneIntersection (PointXYZ line_p1, PointXYZ line_p2, PointXYZ plane_p1, PointXYZ plane_p2, PointXYZ plane_p3, PointXYZ &intersection){

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

PointXYZ FindBBCenter(PointXYZ A, PointXYZ B, PointXYZ C, PointXYZ D){
    PointXYZ res;

    res.x = (A.x + B.x + C.x + D.x)/4;
    res.y = (A.y + B.y + C.y + D.y)/4;
    res.z = (A.z + B.z + C.z + D.z)/4;

    return res;
}

int main(){
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPCDFile("ampora_0.pcd", *cloud);
    boost::shared_ptr<visualization::PCLVisualizer> viewer = SingleCloudVisualizer(cloud);

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

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    viewer->addPointCloud<pcl::PointXYZ> (cloud, "main cloud");
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

//    viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
//    viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
    viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
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

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}