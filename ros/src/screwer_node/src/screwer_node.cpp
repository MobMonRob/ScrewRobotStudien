#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <screwer_node/ScrewerConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <dhbw_screw_localization/PclEye.h>

#include <chrono>

void initializePublisher(ros::NodeHandle& node);
void initializeDynamicReconfigure(dynamic_reconfigure::Server<screwer_node::ScrewerConfig>& server);
void initializeTransformListener(tf::TransformListener& listener);

void reconfigureCallback(screwer_node::ScrewerConfig& config, uint32_t level);
void configurePassThroughParameter(screwer_node::ScrewerConfig& config);
void configureSacSegmentationParameter(screwer_node::ScrewerConfig& config);
void configureEuclideanClusterExtractionParameter(screwer_node::ScrewerConfig& config);
void configureVfhEstimationParameter(screwer_node::ScrewerConfig& config);
void configureSvmClassificationParameter(screwer_node::ScrewerConfig& config);

pcl::PointCloud<pcl::PointXYZ>::Ptr receiveInputPCloud(tf::TransformListener& listener);
void publishResult(std::shared_ptr<PclScrew> screw);

ros::Publisher screw_pose_publisher;
ros::Publisher screw_pcloud_publisher;

PclEyeParameters parameters;

bool running = false;

int main(int argc, char* argv[])
{
    ROS_INFO("Node started.");
    ros::init(argc, argv, "screwer_node");
    ros::NodeHandle node;
    dynamic_reconfigure::Server<screwer_node::ScrewerConfig> dynamic_reconfigure_server;
    tf::TransformListener tf_listener;
    
    initializePublisher(node);
    initializeDynamicReconfigure(dynamic_reconfigure_server);
    initializeTransformListener(tf_listener);
    
    running = true;
    ROS_INFO("Node is up and running.");

    while(ros::ok())
    {
        std::shared_ptr<PclScrew> screw = PclEye::openUp()->useTheseParameters(parameters)->toFindScrewIn(receiveInputPCloud(tf_listener));
        publishResult(screw);
    }

    ros::spin();
    return 0;
}

void initializePublisher(ros::NodeHandle& node)
{
    screw_pose_publisher = node.advertise<geometry_msgs::Pose>("screw_pose", 1);
    screw_pcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("screw_pcloud", 1);
    ROS_INFO("Publisher initialized.");

    return;
}

void initializeDynamicReconfigure(dynamic_reconfigure::Server<screwer_node::ScrewerConfig>& server)
{
    server.setCallback(boost::bind(&reconfigureCallback, _1, _2));
    ROS_INFO("Dynamic-Reconfigure initialized.");

    return;
}

void initializeTransformListener(tf::TransformListener& listener)
{
    if(listener.waitForTransform("/ur_base", "/camera", ros::Time(0), ros::Duration(30.0)))
    {
        ROS_INFO("Transform-Listener initialized.");
    }
    else
    {
        ROS_ERROR("Transform-Listener not initialized.");
    }

    return;
}

void reconfigureCallback(screwer_node::ScrewerConfig& config, uint32_t level)
{
    if(!running) return;

    ROS_INFO("Configuration received.");
    configurePassThroughParameter(config);
    configureSacSegmentationParameter(config);
    configureEuclideanClusterExtractionParameter(config);
    configureVfhEstimationParameter(config);
    configureSvmClassificationParameter(config); 

    return;  
}

void configurePassThroughParameter(screwer_node::ScrewerConfig& config)
{
    parameters.pt.clear();
    
    PclPassThroughParameters pt;
    if(config.pt_x)
    {
        pt.filterFieldName = PclPassThroughFieldName::X;
        pt.filterLimitMin = config.pt_min_x;
        pt.filterLimitMax = config.pt_max_x;
        parameters.pt.push_back(pt);
        ROS_INFO("Passthrough configured. \n\t filterFieldName: %d \n\t filterLimitMin: %f \n\t filterLimitMax: %f", 
                    pt.filterFieldName,
                    pt.filterLimitMin, 
                    pt.filterLimitMax);
    }

    if(config.pt_y)
    {
        pt.filterFieldName = PclPassThroughFieldName::Y;
        pt.filterLimitMin = config.pt_min_y;
        pt.filterLimitMax = config.pt_max_y;
        parameters.pt.push_back(pt);
        ROS_INFO("Passthrough configured. \n\t filterFieldName: %d \n\t filterLimitMin: %f \n\t filterLimitMax: %f", 
                    pt.filterFieldName,
                    pt.filterLimitMin, 
                    pt.filterLimitMax);
    }

    if(config.pt_z)
    {
        pt.filterFieldName = PclPassThroughFieldName::Z;
        pt.filterLimitMin = config.pt_min_z;
        pt.filterLimitMax = config.pt_max_z;
        parameters.pt.push_back(pt);
        ROS_INFO("Passthrough configured. \n\t filterFieldName: %d \n\t filterLimitMin: %f \n\t filterLimitMax: %f", 
                    pt.filterFieldName,
                    pt.filterLimitMin, 
                    pt.filterLimitMax);
    }

    return;
}

void configureSacSegmentationParameter(screwer_node::ScrewerConfig& config)
{
    parameters.sacs.clear();

    PclSacSegmentationParameters sacs;
    sacs.model = static_cast<PclSacModel>(config.sacs_model);
    sacs.method = static_cast<PclSacMethod>(config.sacs_method);
    sacs.distanceThreshold = config.sacs_distance_threshold;
    parameters.sacs.push_back(sacs);
    parameters.sacs.push_back(sacs);
    ROS_INFO("Sac-Segmentation configured. \n\t model: %d \n\t method: %d \n\t distanceThreshold: %f", 
                    sacs.model,
                    sacs.method, 
                    sacs.distanceThreshold);

    return;
}

void configureEuclideanClusterExtractionParameter(screwer_node::ScrewerConfig& config)
{
    parameters.ece.tolerance = config.ece_tolerance;
    parameters.ece.minSize = config.ece_min_size;
    parameters.ece.maxSize = config.ece_max_size;
    ROS_INFO("Euclidean-Cluster-Extraction configured. \n\t tolerance: %f \n\t minSize: %d \n\t maxSize: %d", 
                    parameters.ece.tolerance,
                    parameters.ece.minSize, 
                    parameters.ece.maxSize);

    return;
}

void configureVfhEstimationParameter(screwer_node::ScrewerConfig& config)
{
    parameters.vfhe.normalsRadiusSearch = config.vfhe_normal_radius_search;
    ROS_INFO("Vfh-Estimation configured. \n\t normalsRadiusSearch: %f", 
                    parameters.vfhe.normalsRadiusSearch);

    return;
}

void configureSvmClassificationParameter(screwer_node::ScrewerConfig& config)
{
    parameters.svmc.positive = 1.0;
    parameters.svmc.pathToModel = config.svmc_path_to_model;
    parameters.svmc.training = config.svmc_training;

    ROS_INFO("Svm-Classification configured. \n\t pathToModel: %s", 
                    parameters.svmc.pathToModel.c_str());

    return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr receiveInputPCloud(tf::TransformListener& listener)
{  
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pCloud(nullptr);    
    while(!pCloud.get())
    {
        ROS_INFO("Waiting for frame...");
        pCloud = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth/color/points", ros::Duration(1));
        ros::spinOnce();
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr copyOfInput(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pCloud, *copyOfInput);
    copyOfInput->header.frame_id = "/camera";

    tf::StampedTransform transform;
    listener.lookupTransform("/ur_base", "/camera", ros::Time(0), transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*copyOfInput, *transformedPCloud, transform);
    transformedPCloud->header.frame_id = "/ur_base";

    ROS_INFO("Input Pointcloud received and to /ur_base transformed. \n\t %d Points", transformedPCloud->points.size());
    return transformedPCloud;
}

void publishResult(std::shared_ptr<PclScrew> screw)
{
    if(screw.get())
    {
        ROS_INFO("%d", screw->getPCloud().size());
        pcl::PointCloud<pcl::PointXYZ> screw_pcloud = screw->getPCloud();
        screw_pcloud.header.frame_id = "/ur_base";

        screw_pcloud_publisher.publish(screw_pcloud);
        ROS_INFO("Result published to %s (Pointcloud)", "/screw_pcloud", "/screw_pose");
        return;
    }
    ROS_WARN("No result.");
    return;
}
