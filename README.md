PointCloudProcessor
===================

This program is a real attempt to make a program that combined the Point Cloud Library, OpenNI, OpenCV, and ROS. It's goal is to design modular and reusable classes and files to import data from a kinect or other range camera, then convert this data into a point cloud. This point cloud is displayed in real time to the user, who can then choose to save the current point cloud for editing. When the user saves the current PC, the data gets piped to another program via the ROS publish and subscription functions. This next program will import the PCL data and allow the user to run this data through various filters and statistical outlier removers. 
