#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& message);

ros::Subscriber robot_pose_subscriber;
tf::TransformBroadcaster* tf_broadcaster;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle node;

    robot_pose_subscriber = node.subscribe("/screwer_pose", 1, robotPoseCallback);
    tf_broadcaster = new tf::TransformBroadcaster;

    
    ros::Rate rate(10.0);
    while (node.ok() && argv[1])
    {
        tf::Transform transform_1;
        tf::Transform transform_2;
        tf::Quaternion q;

        transform_1.setOrigin(tf::Vector3(0.0, -0.3, 0.3));
        q.setRPY(0, 0, 0);
        transform_1.setRotation(q);

        transform_2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        q.setRPY(M_PI_4, 0, 0);
        transform_2.setRotation(q);

        ros::Time time = ros::Time::now();

        tf_broadcaster->sendTransform(tf::StampedTransform(transform_1, time, "/ur_base", "/screwer"));
        tf_broadcaster->sendTransform(tf::StampedTransform(transform_2, time, "/screwer", "/camera"));
        rate.sleep();
    }
    
    ros::spin();
}

void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& message)
{
    tf::Transform robot(tf::Quaternion(message->orientation.x, message->orientation.y, message->orientation.z, message->orientation.w), 
                        tf::Vector3(message->position.x, message->position.y, message->position.z));
    
    //TODO: Add static transform between robot and camera
    tf::Transform camera(tf::Quaternion(0, 0, M_PI_4),
                         tf::Vector3(0.0, 0.0, 0.0));
    
    ros::Time time = ros::Time::now();
    tf_broadcaster->sendTransform(tf::StampedTransform(robot, time, "/ur_base", "/screwer"));
    tf_broadcaster->sendTransform(tf::StampedTransform(camera, time, "/screwer", "/camera"));
} 
