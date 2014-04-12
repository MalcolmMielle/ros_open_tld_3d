#include <string.h>

#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include "TLDUtil.h"
#include "Trajectory.h"
#include "Timing.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include "handler3D.hpp"

#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Time filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "open_tld_3d/model.h"
using namespace message_filters;




int main (int argc, char **argv){
	/************************Ros stuff************************/
	ros::init(argc, argv, "Point_Cloud");
	ros::NodeHandle my_node;
	
	ros::NodeHandle priv_node("~");

	//Algo stuff
	Main *main = new Main();
	Config config;
	Gui *gui = new Gui();
	Handler3D *theHandler = new Handler3D();
	
	//ROS param
	int que;
	priv_node.param<int>("queue_size", que, 100);
	//Tracking option
	bool enable3DTracking;
	priv_node.param<bool>("Tracking3D", enable3DTracking, true);
	//Publisher and Subscriber
	ros::Publisher poete=my_node.advertise<geometry_msgs::PolygonStamped>("/tracking2D", 1000);
	ros::Publisher pilote=my_node.advertise<geometry_msgs::PointStamped>("/tracking3D", 1000);
	ros::Subscriber scribe_cloud;
	/****OLD version***/
	
	if(enable3DTracking==true){
		//Do not compare the timestamp for now. A bit random :/
		std::cout<<"FULL track"<<std::endl;
		scribe_cloud = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Handler3D::setCloud, theHandler);
	}
	
	image_transport::ImageTransport it(my_node);
	image_transport::CameraSubscriber scribe_image = it.subscribeCamera("camera/rgb/image", que, boost::bind(&Main::doWork, main, _1));
	

	/*
	//Time synchonisation so we don't have to much calculation =D
	//But it's rather unpredictable... Very bad latency

	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Subscriber<sensor_msgs::PointCloud2> cloud_sub(my_node, "/camera/depth/points_xyzrgb", 100);
	Subscriber<sensor_msgs::Image> image_sub(my_node, "/camera/rgb/image", 100);
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, cloud_sub);
	sync.registerCallback(boost::bind(&Main::doWork, main, _1, _2));*/

	theHandler->pilote=pilote;

	main->gui = gui;
	main->poete = &poete;
	main->handy = theHandler;
	main->pnode = priv_node;
	if(config.init(argc, argv) == PROGRAM_EXIT){
		return EXIT_FAILURE;
	}

	config.configure(main);
	
	//loading ros parameters
	main->loadRosparam();
	
	srand(main->seed);

	if(main->showOutput){
		gui->init();
	}
	
	/***SERVICE**/
	/***INUTILE commentaire***/
	ros::ServiceServer service = my_node.advertiseService("modelisation", &Main::add, main);
	ROS_INFO("Ready to change model");
	
	
	while(ros::ok()){
		ros::spinOnce();
	}

	delete main;
	return EXIT_SUCCESS;
	
}

