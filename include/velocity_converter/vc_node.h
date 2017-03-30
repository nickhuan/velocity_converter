#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h> //cv_bridge::toCvShare
#include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> //imshow

class Velocity_Converter
{
private:
  ros::NodeHandle n;
  ros::Publisher vel_pub;
  ros::Publisher arrow_image_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber image_sub;
  cv::Mat image;
  cv::Mat display_image;
  float velocity_x, velocity_y;
  cv::VideoWriter output_video;

public:
  Velocity_Converter();
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};
