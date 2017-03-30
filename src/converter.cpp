#include <ros/ros.h>
#include "vc_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_converter");
    Velocity_Converter vc;
    ros::spin();
    return 0;
}