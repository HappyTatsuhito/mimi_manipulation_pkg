#include "manipulation_pkg/centroid_detector.h"

class ObjectCentroidDetector{
private:
  ros::Subscriber centroid_calc;
  ros::Subscriber object_image_range_sub;
  ros::Publisher  object_xyz_centroid_pub;
  ros::Publisher  filtered_pub;
  ros::Publisher rviz_marker_pub;
  ros::NodeHandle nh;
  bool receive_range;
  int image_range[4];
public:
  explicit ObjectCentroidDetector(ros::NodeHandle& n):
    nh(n){
    centroid_calc = nh.subscribe("/camera/depth_registered/points",1,&ObjectCentroidDetector::CalcCentroid,this);
    object_image_range_sub = nh.subscribe("/object/image_range",1,&ObjectCentroidDetector::ImageRangeCB,this);
    object_xyz_centroid_pub = nh.advertise<geometry_msgs::Point> ("/object/xyz_centroid", 1);
    filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud",1);
    rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1);
    
    receive_range = FALSE;
  }
  void ImageRangeCB(const object_recognizer::ImageRange input){
    image_range[TOP]   = input.top;
    image_range[BOTTOM]= input.bottom;
    image_range[LEFT]  = input.left;
    image_range[RIGHT] = input.right;
    std::cout << " topic received" << std::endl;
    receive_range = TRUE;
  }
  void CalcCentroid(const sensor_msgs::PointCloud2::Ptr &input){
    //std::cout << "point cloud received" << std::endl;
    if(receive_range != TRUE) return;
    receive_range = FALSE;
    // convert sensor_msgs to pcl
    sensor_msgs::PointCloud2::Ptr input_cloud = input;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
    // remove invalid points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    // perform downsampling for performance 
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_down;
    sor_down.setInputCloud(cloud->makeShared());
    sor_down.setLeafSize(0.01, 0.01, 0.01);
    sor_down.filter(*cloud);
    //remove outliers by statical filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_stat;
    sor_stat.setInputCloud(cloud->makeShared());
    sor_stat.setMeanK(50);
    sor_stat.setStddevMulThresh(1.0);
    sor_stat.setNegative(false);
    sor_stat.filter(*cloud);
    //Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    int nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.7 * nr_points){
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
      if(inliers->indices.size () == 0){
	std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	break;
      }
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);
      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud);
    }
    //calculate color-depth correspondence
    double top   =((double)image_range[TOP]   -239.5)*43/480*M_PI/180;
    double bottom=((double)image_range[BOTTOM]-239.5)*43/480*M_PI/180;
    double left  =((double)image_range[LEFT]  -319.5)*70/640*M_PI/180;
    double right =((double)image_range[RIGHT] -319.5)*70/640*M_PI/180;
	//////
	std::cout << "top,bottom,left,right : "<< top << bottom << left << right << std::endl;
	//////
    //correct the deviation of IR camera and Color camera.
    right=right+(0.61-right)*0.2;
    left=left+(0.61-left)*0.13;
    
    //get centroid of object_point from average value------------------------------------//
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    double sum_point = 0;
    double diff_w = right - left;
    double diff_h = bottom - top;
    double removal_rate = 0.0;
    //object regional cloud
    pcl::PointCloud<pcl::PointXYZRGB> regional_cloud;
    //get number of points in the object region.
    for(int i=0;i<cloud->points.size();i++){
      if(left+diff_w*removal_rate/2 < atan2(cloud->points[i].x,cloud->points[i].z) && atan2(cloud->points[i].x,cloud->points[i].z) < right-diff_w*removal_rate/2 && 
	 top+diff_h*removal_rate/2  < atan2(cloud->points[i].y,cloud->points[i].z) && atan2(cloud->points[i].y,cloud->points[i].z) < bottom-diff_h*removal_rate/2){
	if(cloud->points[i].z < 1.0){
	  sum_point += 1;
	}
	else{
	  //std::cout <<"invalid z is "<<cloud->points[i].z << std::endl;
	}
      }
    }
    if(sum_point == -1){
      std::cout << "object point not exist(^^;)" << std::endl;
      return;
    }
    //resize regional cloud
    int sum_point_i = 0;
    regional_cloud.width = sum_point;
    regional_cloud.height= 1;
    regional_cloud.resize(sum_point);
    //create regional cloud
    for(int i=0;i<cloud->points.size();i++){
      if(left+diff_w*removal_rate/2 < atan2(cloud->points[i].x,cloud->points[i].z) && atan2(cloud->points[i].x,cloud->points[i].z) < right-diff_w*removal_rate/2 && 
	 top+diff_h*removal_rate/2  < atan2(cloud->points[i].y,cloud->points[i].z) && atan2(cloud->points[i].y,cloud->points[i].z) < bottom-diff_h*removal_rate/2){
	if(cloud->points[i].z < 1.0){
	  sum_x += cloud->points[i].x;
	  sum_y += cloud->points[i].y;
	  sum_z += cloud->points[i].z;
	  regional_cloud.points[sum_point_i].x += cloud->points[i].x;
	  regional_cloud.points[sum_point_i].y += cloud->points[i].y;
	  regional_cloud.points[sum_point_i].z += cloud->points[i].z;
	  sum_point_i++;
	}
      }
    }
    //publish regional_cloud
    sensor_msgs::PointCloud2 filtered_cloud;
    filtered_cloud.header.frame_id="filtered_cloud_frame";
    pcl::toROSMsg(regional_cloud,filtered_cloud);
    filtered_cloud.header.frame_id="filtered_cloud_frame";
    filtered_pub.publish(filtered_cloud);
    
    double ave_x = sum_x/sum_point;
    double ave_y = sum_y/sum_point;
    double ave_z = sum_z/sum_point;
    std::cout <<"ave : "<< ave_x <<" "<<ave_y<<" "<<ave_z<<" "<<sum_point<<std::endl;
    
    //transform "/realsense_frame" to "/world" and publish "object/xyz_centroid"-------------------------------------//
    //transform to world from realsense_frame
    geometry_msgs::Point object_point;
    double THETA = -5*M_PI/36;
    double REALSENSE_HEIGHT = 0.98;
    object_point.x = ave_z*cos(THETA)+ave_y*sin(THETA);
    object_point.y = -1*ave_x;
    object_point.z = ave_z*sin(THETA)-ave_y*cos(THETA)+REALSENSE_HEIGHT;
    object_xyz_centroid_pub.publish(object_point);
    std::cout <<"object_point : "<<object_point.x<<" "
	     <<object_point.y<<" "
	     <<object_point.z<<" "<<std::endl;
    //rviz visualization
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/object_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_box";
    marker.id = 0;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = object_point.x;
    marker.pose.position.y = object_point.y;
    marker.pose.position.z = object_point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.color.g = 1.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    rviz_marker_pub.publish(marker);
  }
};

int main (int argc, char** argv){
  ros::init(argc, argv, "object_centroid_detector");
  ros::NodeHandle nh("~");
  
  ObjectCentroidDetector* range_to_centroid = new ObjectCentroidDetector(nh);
  ros::spin();
}
