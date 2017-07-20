//
// Created by aleks on 27/02/17.
//

#ifndef GRADATGEN_GRASPINGDETECTION_H
#define GRADATGEN_GRASPINGDETECTION_H

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


using namespace std;
using namespace pcl;

typedef vector<vector<PointCloud<PointXYZ> > > pcl_database;
typedef vector<PointCloud<Normal> > normal_database;
typedef vector<PointCloud<PointXYZ> > obj_dataset;
typedef vector<vector<PointXYZ> > points_dataset;
typedef vector<vector<vector<PointXYZ> > > grasping_point_dataset;

class GraspingDetection {

public:
    boost::shared_ptr<visualization::PCLVisualizer> GraspVisualizer (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr grasp_cloud, PointCloud<Normal>::ConstPtr normals);
    boost::shared_ptr<visualization::PCLVisualizer> NormalVisualizer (PointCloud<pcl::PointXYZ>::ConstPtr cloud, PointCloud<pcl::Normal>::ConstPtr normals);
    vector<string> ReadDatasetPaths (char *in_file);
    pcl_database ReadPCDFromPaths (vector<string> paths, int pcd_per_obj);
    pcl_database SegmentDatasetPCD (pcl_database pointcloud_dataset);
    obj_dataset MergeAndSubsamplePointCloud (pcl_database pointcloud_dataset);
    normal_database DetectNormalsForGrasping (obj_dataset objects_dataset);
    grasping_point_dataset DetectMomentOfInertiaForGrasping (obj_dataset object_dataset);
};

class MathematicsOperators{
public:
    Eigen::Vector3f PointToPointVector(PointXYZ A, PointXYZ B);
    Eigen::Vector3f Dot(Eigen::Vector3f A, Eigen::Vector3f B);
    Eigen::Vector3f VectorAdd(Eigen::Vector3f A, Eigen::Vector3f B);
    Eigen::Vector3f VectorSub(Eigen::Vector3f A, Eigen::Vector3f B);
    float VectorProduct(Eigen::Vector3f A, Eigen::Vector3f B);
    Eigen::Vector3f VectorMultiplyByScalar(Eigen::Vector3f A, float k);
    Eigen::Vector3f PointToVector(PointXYZ A);
    PointXYZ VectorToPoint(Eigen::Vector3f A);
    bool LinePlaneIntersection (PointXYZ line_p1, PointXYZ line_p2, PointXYZ plane_p1, PointXYZ plane_p2, PointXYZ plane_p3, PointXYZ &intersection);
    float PointToPointEuclideanDistance(PointXYZ A, PointXYZ B);
    bool GraspingPointIsValid (vector<PointXYZ> inter, vector<PointXYZ> &check);

};


#endif //GRADATGEN_GRASPINGDETECTION_H
