#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <queue>
#include <iostream>
#include <stdio.h>
#include <string>

typedef std::queue<std::string> StringQueue;
typedef pcl::PointCloud<pcl::PointXYZ> PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudPtr;

StringQueue getFileNamesInDictionary(const char* path);

ros::Publisher ply_publisher;

int main(int argc, char* argv[])
{
    ROS_INFO("Node started.");
    ros::init(argc, argv, "ply_reader_node");
    ros::NodeHandle node;
    ply_publisher = node.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1000);

    std::string ply_path = argv[1];
    StringQueue plys = getFileNamesInDictionary(ply_path.c_str());

    while(ros::ok() && plys.size() > 0)
    {
        PCloudPtr pCloud(new PCloud());
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(plys.front(), *pCloud) == -1)
        {
            ROS_ERROR("Could not read *.ply-file.");
            exit(0);
        }

        pCloud->header.frame_id = "/camera";
        ply_publisher.publish(*pCloud);
        plys.pop();
    }

    ros::spin();
    return 0;
}

StringQueue getFileNamesInDictionary(const char* path)
{
    StringQueue ply_paths;
    DIR* dirp = opendir(path);
    struct dirent * dp;
    while((dp = readdir(dirp)) != NULL) 
    {
        std::string filename = dp->d_name;
        ply_paths.push(path + filename);
    }
    closedir(dirp);
    ply_paths.pop();
    ply_paths.pop();
    return ply_paths;
}
