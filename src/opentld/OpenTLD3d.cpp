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

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"


#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main (int argc, char **argv){
	
	Main *main = new Main();
	Config config;
//	ImAcq *imAcq = imAcqAlloc();
	Gui *gui = new Gui();
	handler3D *theHandler = new handler3D();
	
	ros::init(argc, argv, "point_cloud");
	ros::NodeHandle nh;
	std::string topic = nh.resolveName("point_cloud");
	uint32_t queue_size = 1;
	
	//Publisher and Subscriber
	ros::Publisher poete=nh.advertise<geometry_msgs::PolygonStamped>("/2D_tracking", 1000);
	ros::Publisher pilote=nh.advertise<geometry_msgs::PoseStamped>("/3D_tracking", 1000);
	ros::Subscriber scribe = nh.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 100, &Main::doWork, main);

	theHandler->pilote=pilote;

	main->gui = gui;
	//main->imAcq = imAcq;
	main->poete = &poete;
	main->handy = theHandler;
	if(config.init(argc, argv) == PROGRAM_EXIT)
	{
		return EXIT_FAILURE;
	}

	config.configure(main);
	
	//loading ros parameters
	main->loadRosparam();
	
	srand(main->seed);

	if(main->showOutput){
		gui->init();
	}
	while(ros::ok()){
		ros::spinOnce();
	}

	delete main;
	return EXIT_SUCCESS;
	
}

