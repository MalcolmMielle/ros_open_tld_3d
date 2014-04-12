/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/*
 * main.h
 *
 *  Created on: Nov 18, 2011
 *      Author: Georg Nebehay
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "handler3D.hpp"
#include "Trajectory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Duration.h"
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Image.h>
#include "TLD.h"
#include "ImAcq.h"
#include "Gui.h"


#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#include "open_tld_3d/model.h"


using namespace tld;
using namespace cv;

enum Retval
{
    PROGRAM_EXIT = 0,
    SUCCESS = 1
};

class Main
{
public:
	tld::TLD *tld;
//	ImAcq *imAcq;
	tld::Gui *gui;
	bool showOutput;
	bool showTrajectory;
	int trajectoryLength;
	const char *printResults;
	const char *saveDir;
	double threshold;
	bool showForeground;
	bool showNotConfident;
	bool selectManually;
	int *initialBB;
	bool reinit;
	bool exportModelAfterRun;
	bool loadModel;
	std::string modelPath;
	std::string modelExportFile;
	int seed;
	//New
	bool keyboardControl;
	bool flag;
	bool reuseFrameOnce;
	bool skipProcessingOnce;
	FILE *resultsFile;
	Trajectory trajectory;
	double time_constant;
	//3D
	bool enable3DTracking;
	Handler3D *handy;

	//ROS
	ros::Publisher *poete;
	ros::NodeHandle pnode;

	Main()
	{
		tld = new tld::TLD();
		
		/*if(!ros::param::get("/OpenTLD/Graphical_interface", showOutput)){
			showOutput=true;
		}*/

		printResults = NULL;
		saveDir = ".";
		threshold = 0.5;
		showForeground = 0;

		showTrajectory = false;
		trajectoryLength = 0;

		selectManually = 0;

		initialBB = NULL;
		showNotConfident = true;

		reinit = 0;

		loadModel = false;
		keyboardControl = true;
		exportModelAfterRun = false;
		modelExportFile = "model";
		seed = 0;
		//News
		flag=true;
		reuseFrameOnce = false;
		skipProcessingOnce = false;
		resultsFile = NULL;
	}

    ~Main()
    {
        delete tld;
    }
    
    	void doWork(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& cloudy);
	void doWork(const sensor_msgs::ImageConstPtr& msg);
	void publish(cv::Rect *currBB);
	void loadRosparam();
	void Gui(Mat& img, Mat& grey);
	bool add(open_tld_3d::model::Request  &req, open_tld_3d::model::Response &res);
    
};

#endif /* MAIN_H_ */
