#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <queue>
#include <iostream>
#include <stdio.h>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ> PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudPtr;

long counter = 0;

void callback(const PCloud::ConstPtr& msg)
{
    pcl::io::savePLYFile<pcl::PointXYZ>("/home/lstern/ScrewRobotStudien/ros/" + std::to_string(counter++) + ".ply", *msg);
}

int main(int argc, char* argv[])
{
    ROS_INFO("Node started.");
    ros::init(argc, argv, "ply_writer_node");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe<PCloud>("/screw_pcloud", 1, callback);

    ros::spin();
    return 0;
}