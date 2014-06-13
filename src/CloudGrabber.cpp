/** GrabberClass.cpp
* @Author - John H Allard. 6/12/2014. Created at Harvey Mudd as part of the 2014 CS REU under Dr. Zach Dodds
* @ Description - This is the implementation of the CloudGrabber class. This file will implement the member functions of this class,
*       which connects the program to the kinect camera via the openNI drivers, converts the incoming data to a Point Cloud, visualized the point cloud
*       for the user to view, then gives the user the ability to save the current point cloud and to publish this data to another program, which allows the
*       user to apply various transformations, filters, and other actions on the current point cloud.
*/

// User Defined Includes
#include "CloudGrabber.h"


// PCL Includes 

// ROS Includes

// STD C++ Icnludes

CloudGrabber::CloudGrabber()
{
    //this->cloudptr = new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //this->fallbackCloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    this->filesSaved = 0;
    this->saveCloud = false;
    this->noColor = false;

    openniGrabber = new OpenNIGrabber();
    if (!openniGrabber)
        PCL_ERROR("Could not grab data from camera");

    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f2( boost::bind( &CloudGrabber::grabberCallback, this, _1 ) );
    openniGrabber->registerCallback(f2);

    viewer = createViewer();

    openniGrabber->start();
}

// Creates, initializes and returns a new viewer, which is a window that will display the incoming PC data
boost::shared_ptr<visualization::CloudViewer> CloudGrabber::createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("Kinect Camera Cloud Viewer"));
    v->registerKeyboardCallback(&CloudGrabber::keyboardEventOccurred, *this, (void*)& viewer);

    return v;
}

    // For detecting when SPACE is pressed.
void CloudGrabber::keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}


void CloudGrabber::grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if (!this->viewer->wasStopped())
        this->viewer->showCloud(cloud);

    if (saveCloud)
    {
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();
        if (io::savePCDFile(filename, *cloud, true) == 0)
        {
            filesSaved++;
            cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;
    }
}


boost::shared_ptr<visualization::CloudViewer> CloudGrabber::getViewer()
{

    return this->viewer;
}