/** GrabberClass.cpp
* @Author - John H Allard. 6/12/2014. Created at Harvey Mudd as part of the 2014 CS REU under Dr. Zach Dodds
* @ Description - This is the implementation of the CloudGrabber class. This file will implement the member functions of this class,
*       which connects the program to the kinect camera via the openNI drivers, converts the incoming data to a Point Cloud, visualized the point cloud
*       for the user to view, then gives the user the ability to save the current point cloud and to publish this data to another program, which allows the
*       user to apply various transformations, filters, and other actions on the current point cloud.
*/

// User Defined Includes
#include "CloudGrabber.h"


CloudGrabber::CloudGrabber()
: cloudptr(new pcl::PointCloud<pcl::PointXYZRGBA>), // initialize our PointCloud ptr 
node(new ros::NodeHandle)
{
    // initialize members
    this->filesSaved = 0;
    this->saveCloud = false;
    this->noColor = false;
    this->publishCurrent = false;
    this->visualize = true; // make it so we visualize the incoming data by default
    this->PUB_NAME = "chatter";

    openniGrabber = new OpenNIGrabber();
    if (!openniGrabber)
        PCL_ERROR("Could not grab data from camera");

    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> grabber_cb( boost::bind( &CloudGrabber::grabberCallback, this, _1 ) );
    openniGrabber->registerCallback(grabber_cb);

}


// start the grabbing of data from the camera source and visualization of the point clouds
void CloudGrabber::startFeed()
{
    // create the viewing window for the incoming point cloud data
    viewer = createViewer();
    // start grabbing the point cloud data through OpenNI
    openniGrabber->start();
}

// this function is called to set up a ROS publisher and publish important PCL data coming from the openniGrabber\
// @param n - the ros::NodeHandle object 
void CloudGrabber::startPublishing()//ros::NodeHandle n)
{
    // set up our publisher to output PCLPointCloud2 messages under the name @field PUB_NAME
    this->publisher = node->advertise<pcl::PCLPointCloud2&>(this->PUB_NAME, 1000);

    // set to publish 10 times a second, should change this to be a variable submitted by the user
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        // if the user set the publish flag
        // TODO - Change it so that we publish the messages even without user input, it shouldn't slow us down too much
        // but it will simplify future processes
        if(this->publishCurrent)
        {
            pcl::PCLPointCloud2 tempcloud;
            //Input of the above cloud and the corresponding output of cloud_pcl
            pcl::toPCLPointCloud2(*cloudptr, tempcloud);
            publisher.publish(tempcloud);
            this->publishCurrent = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Creates, initializes and returns a new viewer, which is a window that will display the incoming Point Cloud data
boost::shared_ptr<visualization::CloudViewer> CloudGrabber::createViewer()
{
    // create our viewer object
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("Kinect Camera Cloud Viewer"));

    // register the keyboard callback
    v->registerKeyboardCallback(&CloudGrabber::keyboardEventOccurred, *this, (void*)& viewer);

    return v;
}

    // For detecting when SPACE is pressed.
void CloudGrabber::keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
    // if the user wants to save the current point cloud, set the save flag
    if (event.getKeySym() == "s" && event.keyDown())
        saveCloud = true;

    // if the user presses v, we toggle the visualize flag. 
    // flag == true means we show the incoming point clouds, flag == flase means we don't update the visualizer (sames memory and processor clocks)
    if(event.getKeySym() == "v" && event.keyDown())
        visualize =! visualize;

    // if the user presses the 'p' key we go ahead and publish the current point cloud stored in cloudptr through our publisher object
    // for othe programs to pick up and work on
    if(event.getKeySym() == "p" && event.keyDown())
        this->publishCurrent = true;


}


void CloudGrabber::grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if (!this->viewer->wasStopped() && visualize == true)
    {
        *cloudptr = *cloud;
        this->viewer->showCloud(cloudptr);
    }

    if (saveCloud)
    {
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();
        if (io::savePCDFile(filename, *cloudptr, true) == 0)
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

// get the name of the publisher
std::string CloudGrabber::getPublisherName()
{
    return this->PUB_NAME;
}

// get the ROS publisher object
ros::Publisher & CloudGrabber::getPublisher()
{
    return this->publisher;
}

ros::NodeHandle * CloudGrabber::getNodeHandle()
{
    return node;
}