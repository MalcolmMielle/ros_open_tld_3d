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
#include <geometry_msgs/PoseStamped.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class handler3D{
	public : 
	ros::Publisher pilote;
	geometry_msgs::PoseStamped poseSt;
	handler3D(){
		
	};
	
	void conversionFROMrosmsg(const sensor_msgs::PointCloud2ConstPtr& cloudy, cv::Mat& result, pcl::PointCloud<pcl::PointXYZRGBA>* cloud);
	void deepness(cv::Rect *currBB, pcl::PointCloud<pcl::PointXYZRGBA>* cloud);
	void publish();

};

#endif
