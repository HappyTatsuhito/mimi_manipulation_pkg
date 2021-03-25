#include <ros/ros.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
// custom srv
#include <mimi_manipulation_pkg/DetectDepth.h>

class ThreeDimensionalPositionEstimator
{
public:
  ThreeDimensionalPositionEstimator();
  ~ThreeDimensionalPositionEstimator(){};

private:
  ros::NodeHandle nh;
  /* -- topic -- */
  ros::Subscriber realsense_subscriber;
  /* -- service -- */
  ros::ServiceServer estimate_server;
  /* -- param -- */
  float realsense_height = 0.0;
  /* -- member variables -- */
  sensor_msgs::ImageConstPtr depth_image;
  /* -- member functions --*/
  void realSenseCB(const sensor_msgs::ImageConstPtr& ros_image);
  bool convertImage(const sensor_msgs::ImageConstPtr& input_image, cv::Mat &output_image);
  bool getDepth(mimi_manipulation_pkg::DetectDepth::Request &req, mimi_manipulation_pkg::DetectDepth::Response &res);
};

ThreeDimensionalPositionEstimator::ThreeDimensionalPositionEstimator() : nh("")
{
  realsense_subscriber = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ThreeDimensionalPositionEstimator::realSenseCB, this);
  estimate_server = nh.advertiseService("/detect/depth", &ThreeDimensionalPositionEstimator::getDepth, this);
  nh.getParam("/mimi_specification/RealSense_Height", realsense_height);
}

void ThreeDimensionalPositionEstimator::realSenseCB(const sensor_msgs::ImageConstPtr& ros_image){
  depth_image = ros_image;
}

bool ThreeDimensionalPositionEstimator::convertImage(const sensor_msgs::ImageConstPtr& input_image, cv::Mat &output_image){
  cv_bridge::CvImagePtr cv_ptr;
  try{
	// ros image msg -> cv_bridge -> cv::Mat
	cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::TYPE_16UC1);
	output_image = cv_ptr->image;
	
	/* 画像出力
	cv::imshow("Image_window", output_image);
	cv::waitKey(0);
	cv::destroyWindow("Image_window");
	*/
  }
  catch (cv_bridge::Exception& e){
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return false;
  }
  return true;
}

bool ThreeDimensionalPositionEstimator::getDepth(mimi_manipulation_pkg::DetectDepth::Request &req, mimi_manipulation_pkg::DetectDepth::Response &res){
  geometry_msgs::Point object_point;
  cv::Mat cv_image;
  sensor_msgs::ImageConstPtr current_depth_image = depth_image;

  bool convert_result = convertImage(current_depth_image, cv_image);

  if(!convert_result){
	res.centroid_point.x = std::numeric_limits<float>::quiet_NaN();
	res.centroid_point.y = std::numeric_limits<float>::quiet_NaN();
	res.centroid_point.z = std::numeric_limits<float>::quiet_NaN();
	return false;
  }

  float distance = cv_image.at<u_int16_t>(req.center_x, req.center_y);
  ROS_INFO("%f", distance);

  float theta_y, theta_z, centroid_x, centroid_y, centroid_z;
	
  theta_y = ((req.center_y-320)*53.6/640)/180*M_PI;
  theta_z = (-1*(req.center_x-240)*41.0/480)/180*M_PI;

  //角度：0
  centroid_x = distance;
  centroid_y = -1 * distance * tan(theta_y);
  centroid_z = distance * tan(theta_z);
  
  //角度：30
  centroid_x = centroid_x * cos(M_PI*30/180) + centroid_z * sin(M_PI*30/180);
  centroid_z = centroid_z - distance*sin(M_PI*30/180);

  res.centroid_point.x = centroid_x / 1000;
  res.centroid_point.y = centroid_y / 1000;
  res.centroid_point.z = centroid_z / 1000 + realsense_height;
  ROS_INFO("x:%f, y:%f, z:%f", res.centroid_point.x, res.centroid_point.y, res.centroid_point.z);
  
  return true;  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "three_dimensional_position_estimator");

  ThreeDimensionalPositionEstimator three_dimensional_position_estimator;
  
  ROS_INFO("Ready");
  ros::spin();
  return 0;
}
