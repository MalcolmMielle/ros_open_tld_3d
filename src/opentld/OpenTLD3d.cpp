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

int main (int argc, char **argv){

	/************************Ros stuff************************/
	ros::init(argc, argv, "Point_Cloud");
	ros::NodeHandle my_node;
	ros::Rate loop_rate(10);
	
	//Algo stuff
	Main *main = new Main();
	Config config;
	Gui *gui = new Gui();
	Handler3D *theHandler = new Handler3D();
	
	//Publisher and Subscriber
	ros::Publisher poete=my_node.advertise<geometry_msgs::PolygonStamped>("/2D_tracking", 1000);
	ros::Publisher pilote=my_node.advertise<geometry_msgs::PointStamped>("/3D_tracking", 1000);
	
	//Loading source.
	std::string source;
	ros::param::param<std::string>("/OpenTLD/source",source, "camera/rgb/image"); 

	//Comparing timestamp myself. A bit random :/
	ros::Subscriber scribe_cloud = my_node.subscribe<sensor_msgs::PointCloud2> ("camera/depth/points_xyzrgb", 1, &Handler3D::setCloud, theHandler);
	//ros::Subscriber scribe=my_node.subscribe<sensor_msgs::Image>( "camera/rgb/image",1, &Main::doWork, main);
	
	image_transport::ImageTransport it(my_node);
	image_transport::CameraSubscriber scribe_image = it.subscribeCamera(source, 1, boost::bind(&Main::doWork, main, _1));

	//Time synchonisation so we don't have to much calculation =D
	/*message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth/points_xyzrgb", 100);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image", 100);
	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(cloud_sub, image_sub, 10);
	sync.registerCallback(boost::bind(&Main::doWork, main, _1, _2));*/

	theHandler->pilote=pilote;

	main->gui = gui;
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
	//std::cout<<"spin"<<std::endl;
		ros::spinOnce();
	}

	delete main;
	return EXIT_SUCCESS;
	
}

