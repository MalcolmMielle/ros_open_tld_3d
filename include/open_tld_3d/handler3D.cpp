#include "handler3D.hpp"


void Handler3D::setCloud(const sensor_msgs::PointCloud2ConstPtr& cloudy){
	pcl::fromROSMsg(*cloudy, *cloud);
}

void Handler3D::tracking(cv::Rect *currBB){
	if(currBB!=NULL){
		int x=currBB->x+(currBB->width/2);
		int y=currBB->y+(currBB->height/2);
		pcl::PointXYZRGBA point(cloud->at(x,y));
		if(!isnan(point.x) && !isnan(point.y) && !isnan(point.z)){
			(*this).publish(point);
		}
	}
}

void Handler3D::conversionFROMrosmsg(const sensor_msgs::PointCloud2ConstPtr& cloudy, cv::Mat& result, pcl::PointCloud<pcl::PointXYZRGBA>* cloudin){

	pcl::fromROSMsg(*cloudy, *cloudin);	
	if (cloudin->isOrganized()) {
	    result = cv::Mat(cloudin->height, cloudin->width, CV_8UC3);
	    
		  for (int h=0; h<result.rows; h++) {
		      for (int w=0; w<result.cols; w++) {
		          pcl::PointXYZRGBA point = cloudin->at(w, h);

		          Eigen::Vector3i rgb = point.getRGBVector3i();

		          result.at<cv::Vec3b>(h,w)[0] = rgb[2];
		          result.at<cv::Vec3b>(h,w)[1] = rgb[1];
		          result.at<cv::Vec3b>(h,w)[2] = rgb[0];
		      }
		  }
	}
	else{
		std::cerr << "Cloud not organized, can't apply the function" << std::endl;
	}
}

void Handler3D::publish(pcl::PointXYZRGBA& point){

	geometry_msgs::PointStamped pt;
	pt.point.x=point.x;
	pt.point.y=point.y;
	pt.point.z=point.z;
	pt.header.stamp=ros::Time::now();
	pt.header.frame_id="camera";
	(*this).pilote.publish(pt);
}
