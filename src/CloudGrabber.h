#ifndef CLOUDGRABBER_H_
#define CLOUDGRABBER_H_

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
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>


 // ROS Related Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

using namespace pcl;
using namespace std;

class CloudGrabber
{

private:
    // Our Cloud Object that we work with
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr;

    boost::shared_ptr<visualization::CloudViewer> viewer;  // Our visualizer object, used to visualize the point clouds                
    Grabber* openniGrabber;                                // Our grabber object, grabs video feed from kinect camera and sends it to @function grabberCallback 
                                                                                                              
    unsigned int filesSaved;                            // incremented to save different files to disc
    bool visualize, saveCloud, noColor, publishCurrent; // Helper variables to denote program states

    // ROS realted stuff
    ros::Publisher publisher;                // our PointCloud publisher object
    std::string PUB_NAME;                    // the broadcasting name of our publisher for ROS subscribers to find
	
	
public:

    // Constructor, takes no arguments and needs to arguments
    CloudGrabber();

    // Creates, initializes and returns a new viewer.
    boost::shared_ptr<visualization::CloudViewer> createViewer();

    // start the data feed and visualization from the camera source
    void startFeed();

    // called to start publishing data from the kinect
    void startPublishing(ros::NodeHandle);

    // For detecting when keyboard command are issues, allows the user to save, toggle visualization, and publish current point cloud data
    void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing);

    // this function is called everytime there is new data
    void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud);

    // GET & SET Functions
    boost::shared_ptr<visualization::CloudViewer> getViewer(); // get the viewer object

    std::string getPublisherName(); // get the name of the publisher

    ros::Publisher getPublisher(); // get the ROS publisher object
	
};




#endif /* GRABBERCLASS_H_ */
