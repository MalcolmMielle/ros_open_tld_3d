
#include "ros/ros.h"
#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"

#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void callback(const sensor_msgs::PointCloud2ConstPtr& cloudy){

	std::cout<<"helow word"<<std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::fromROSMsg(*cloudy, cloud);	
	
	cv::Mat result; 

	if (cloud.isOrganized()) {
		std::cout<<"Safe"<<std::endl;
	    result = cv::Mat(cloud.height, cloud.width, CV_8UC3);
	    
		  for (int h=0; h<result.rows; h++) {
		      for (int w=0; w<result.cols; w++) {
		          pcl::PointXYZRGBA point = cloud.at(w, h);

		          Eigen::Vector3i rgb = point.getRGBVector3i();

		          result.at<cv::Vec3b>(h,w)[0] = rgb[2];
		          result.at<cv::Vec3b>(h,w)[1] = rgb[1];
		          result.at<cv::Vec3b>(h,w)[2] = rgb[0];
		      }
		  }
	}
}


int main (int argc, char **argv){
	ros::init(argc, argv, "point_cloud");
	ros::NodeHandle nh;
	std::string topic = nh.resolveName("point_cloud");
	uint32_t queue_size = 1;
	
	// callback signature
	//void callback(const sensor_msgs::PointCloud2ConstPtr&);

	// to create a subscriber, you can do this (as above):
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", queue_size, callback);
	
	while(ros::ok()){
		ros::spinOnce();
	}
}

