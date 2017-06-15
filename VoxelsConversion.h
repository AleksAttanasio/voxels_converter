//
// Created by osboxes on 06/06/17.
//

#ifndef VOXELSCONVERSION_H
#define VOXELSCONVERSION_H

#include <ctime>
#include <iostream>
#include <math.h>
#include <sstream>
#include <vector>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;

typedef vector<PointCloud<PointXYZ> > Clusters;
typedef vector<vector <vector< int > > > IntMatrix;
typedef vector<vector<vector <vector< int > > > > IntMatrix4D;
typedef Eigen::Matrix<float,4,1> FloatMatrixCOM;


class VoxelsConversion {

    int cont_;
    string folder_;
public:

    int side_matrix_;
    /*****************/
    /* Visualization */
    /*****************/

    // Viewer for two pointclouds
    boost::shared_ptr<visualization::PCLVisualizer> CloudVisualizer (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr second_cloud);
    // Viewer for a single pointclouds
    boost::shared_ptr<visualization::PCLVisualizer> SingleCloudVisualizer (PointCloud<PointXYZ>::ConstPtr first_cloud);

    /**************/
    /* Evaluation */
    /**************/

    // Evaluate the center of mass of a PCL
    FloatMatrixCOM getCenterOfMass (PointCloud<PointXYZ> cloud);

    // Segment the point cloud using Region Growing Segmentation method
    Clusters RegionGrowingSegment(string name_cloud, float smooth_th = 4.0, float curv_th = 0.5, float min_cluster_size = 200);

    PointCloud<PointXYZ>::Ptr generateCube(float x, float y, float z);

    PointCloud<PointXYZ>::Ptr rotatePointCloud(PointCloud<PointXYZ>::Ptr cloud, float rot_x, float rot_y, float rot_z);

    // Given the transformation matrix "trans" the PCL can be translated
    PointCloud<PointXYZ>::Ptr translatePointCloud(PointCloud<PointXYZ>::Ptr cloud, FloatMatrixCOM trans);

    // Generates the matrix with voxels
    IntMatrix getMatrix(PointCloud<PointXYZ>::Ptr cloud);

    // Generates the .txt file with voxels
    void writeMat(IntMatrix mat, const std::string file_name);

};


#endif //VOXELSCONVERSION_H
