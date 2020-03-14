#ifndef IMAGE_RANGE_TO_XYZ_CENTROID_H
#define IMAGE_RANGE_TO_XYZ_CENTROID_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include "/home/nvidia/catkin_ws/devel/include/object_recognizer/ImageRange.h"
#include "/home/demlab/catkin_ws/devel/include/manipulation_pkg/ImageRange.h"
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
//include filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
//include SAC
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//include others
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <vector> 
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#define TRUE 1
#define FALSE 0

#define TOP    0
#define BOTTOM 1
#define LEFT   2
#define RIGHT  3

#define X 0
#define Y 1
#define Z 2

#endif
