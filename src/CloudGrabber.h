#ifndef CLOUDGRABBER_H_
#define CLOUDGRABBER_H_

/*
 * GrabberClass.h
 *
 * @ Author - John H Allard, Created at Harvey Mudd under Dr. Zachary Dodds for the CS REU 2014 Program
 * @Description - This class serves as a means to simplify and abstract some difficulties involving ROS and the PCL Library away from the user.
 * Including this class into a project allows the programmer to detect a kinect camera connected via USB, continuously gather data from that kinect camera,
 * converting that data into a Point Cloud object, displaying that data in a continuous '3D' video-feed on the users screen, saving specific '3D-frames' to
 * file, and publishing the incoming '3D' frame at specific time intervals via the ROS publisher and subscription libraries, all with just an object 
 * instantiation, a CloudGrabber::start function call to start the data capture and video feed, and a ClodGrabber::startPublisher call to start the publishing of
 * data for other programs to intercept and do with what they wish.
 *
 * Important Notes 
 * - If you wish to use the ROS features with this class you must first make a 'roscore' call inside a terminal. This will start the ROS 
 *   drivers and allow our class to publish data.
 *
 * - The CmakeFile must include the following lines (or equivilent)
 *       include_directories(${PCL_INCLUDE_DIRS})
 *       link_directories(${PCL_LIBRARY_DIRS})
 *       add_definitions(${PCL_DEFINITIONS})
 *       find_package( catkin REQUIRED COMPONENTS roscpp )
 *       target_link_libraries (PointCloudProcessor ${PCL_LIBRARIES})
 *       target_link_libraries (PointCloudProcessor ${catkin_LIBRARIES})
 *
 * - This class has been designed around the ROS-Hydro Desktop Build, for a step-by-step tutorial on how to set up your computer to be compatible with 
*    ROS, PCL, and OpenNI, go here https://www.cs.hmc.edu/twiki/bin/view/Robotics/AlexPage
 *
 */


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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr;   // A ptr to our current Point Cloud 'frame' we obtained from the openniGrabber,
                                                        // it is set by intercepting the incoming data in the GrabberCallback function before it is displayed

    boost::shared_ptr<visualization::CloudViewer> viewer;  // Our visualizer object, used to visualize the point clouds                
    Grabber* openniGrabber;                                // Our grabber object, grabs video feed from kinect camera and sends it to @function grabberCallback 
                                                                                                              
    unsigned int filesSaved;                            // incremented to save different files to disc
    bool visualize, saveCloud, noColor, publishCurrent; // Helper variables to denote program states

    // ROS realted stuff
    ros::Publisher publisher;                // our PointCloud publisher object
    std::string PUB_NAME;                    // the broadcasting name of our publisher for ROS subscribers to find

    ros::NodeHandle * node;                       // Handle to the Node for our ROS publisher and/or subscription servies
	
	
public:

    // Constructor, takes no arguments and needs to arguments
    CloudGrabber();

    // Creates, initializes and returns a new viewer.
    boost::shared_ptr<visualization::CloudViewer> createViewer();

    // start the data feed and visualization from the camera source
    void startFeed();

    // called to start publishing data from the kinect
    void startPublishing();

    // GET & SET Functions
    boost::shared_ptr<visualization::CloudViewer> getViewer(); // get the viewer object

    std::string getPublisherName(); // get the name of the publisher

    ros::Publisher & getPublisher(); // get the ROS publisher object

    ros::NodeHandle * getNodeHandle(); // get our ROS Node Handle

protected:
    // For detecting when keyboard command are issues, allows the user to save, toggle visualization, and publish current point cloud data
    void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing);

    // this function is called everytime there is new data
    void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud);
	
};




#endif /* GRABBERCLASS_H_ */
