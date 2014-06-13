/*
 * GrabberClass.h
 *
 * @ Author - john H Allard, Created at Harvey Mudd under Dr. Zachary Dodds for the CS REU 2014 Program
 * A Description - This class holds together all of the data fields and common functionality needed to obtain data from a kinnect
 * and two send this data over to another program to be processed (filtered, feature recognition, etc).
 */

 // Project File Includes

 // Standard C++ Includes
#include <iostream>
#include <sstream>

 // PCL related includes
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

 // ROS Related Includes
#include "ros/ros.h"
#include "std_msgs/String.h"


#ifndef CLOUDGRABBER_H_
#define CLOUDGRABBER_H_

using namespace pcl;
using namespace std;

class CloudGrabber
{

private:
    // Our Cloud Objects that we work with
    //PointCloud<PointXYZRGBA>::Ptr * cloudptr;     // A cloud that will store color info.
    //PointCloud<PointXYZ>::Ptr * fallbackCloud;    // A fallback cloud with just depth data.

    // Our visualizer object, used to visualize the point clouds
    boost::shared_ptr<visualization::CloudViewer> viewer;                 
    Grabber* openniGrabber; 
                                                  
    unsigned int filesSaved;                                               // incremented to save different files to disc
    bool saveCloud, noColor;                                              // Helper variables to denote program states

    // ROS realted stuff
	
	
public:

    // Constructor, takes no arguments and needs to arguments
    CloudGrabber();

    // Creates, initializes and returns a new viewer.
    boost::shared_ptr<visualization::CloudViewer> createViewer();

    // start the data feed and visualization from the camera source
    void startFeed();

    // called to start publishing data from the kinect
    void startPublishing(ros::NodeHandle);

    // For detecting when SPACE is pressed.
    void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing);

    // this function is called everytime there is new data
    void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud);

    // GET & SET Functions
    boost::shared_ptr<visualization::CloudViewer> getViewer();
	
};




#endif /* GRABBERCLASS_H_ */
