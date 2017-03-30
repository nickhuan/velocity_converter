#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "vc_node.h"

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_converter");
    Velocity_Converter vc;
    ros::spin();
    return 0;
}