#ifndef DHANDLER_HPP
#define DHANDLER_HPP
//ros includes
#include "ros/ros.h"

//openCV includes

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Handler3D{
	public : 
	ros::Publisher pilote;
	ros::Time time_stamp;
	pcl::PointCloud<pcl::PointXYZRGBA>* cloud;
	Handler3D() : time_stamp(ros::Time::now()), cloud(new pcl::PointCloud<pcl::PointXYZRGBA>){
	};
	~Handler3D(){delete cloud;}
	void setCloud(const sensor_msgs::PointCloud2ConstPtr& cloudy);
	void conversionFROMrosmsg(const sensor_msgs::PointCloud2ConstPtr& cloudy, cv::Mat& result, pcl::PointCloud<pcl::PointXYZRGBA>* cloudin);
	void publish(pcl::PointXYZRGBA& pt);
	void tracking(cv::Rect *currBB);
};

#endif
