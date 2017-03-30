#include "vc_node.h"

// a helper function to draw arrow for velocity display
void drawArrow(cv::Mat &image, cv::Point p, cv::Point q, cv::Scalar color, int arrowMagnitude = 9, int thickness = 1, int line_type = 8, int shift = 0)
{
    //Draw the principle line
    line(image, p, q, color, thickness, line_type, shift);
    // const double PI = 3.141592653;
    //compute the angle alpha
    double angle = atan2((double)p.y - q.y, (double)p.x - q.x);
    //compute the coordinates of the first segment
    p.x = (int)(q.x + arrowMagnitude * cos(angle + CV_PI / 4));
    p.y = (int)(q.y + arrowMagnitude * sin(angle + CV_PI / 4));
    //Draw the first segment
    line(image, p, q, color, thickness, line_type, shift);
    //compute the coordinates of the second segment
    p.x = (int)(q.x + arrowMagnitude * cos(angle - CV_PI / 4));
    p.y = (int)(q.y + arrowMagnitude * sin(angle - CV_PI / 4));
    //Draw the second segment
    line(image, p, q, color, thickness, line_type, shift);
}

Velocity_Converter::Velocity_Converter()
{
    cv::namedWindow("Velocity Estimation", cv::WINDOW_NORMAL); // Create a window for display.
    // // // record video
    // const std::string NAME = "/home/zhonghuan/velocity_estimation";
    // cv::Size S = cv::Size(1280, 1024);
    // output_video.open(NAME, -1, 30.0, S, true);
    // subscribe and publish topics
    vel_pub = n.advertise<geometry_msgs::Twist>("/converter/velocity", 100); // velocity output
    arrow_image_pub = n.advertise<sensor_msgs::Image>("/converter/image_arrow", 30); //display image output
    image_sub = n.subscribe("/camera/image_raw", 30, &Velocity_Converter::imageCallback, this); // image input
    pose_sub = n.subscribe("/st_ntu_vo/pose", 30, &Velocity_Converter::poseCallback, this); // visual odometry input
}

void Velocity_Converter::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    static float previous_x, previous_y;
    static ros::Time previous_time;
    static bool initialized = 0;

    float current_x, current_y;
    ros::Time current_time;
    float duration;
    geometry_msgs::Twist vel_msg;

    current_x = msg.pose.pose.position.x;
    current_y = msg.pose.pose.position.y;
    current_time = msg.header.stamp;
    // std::cout << "msg.header.stamp: " << msg.header.stamp << std::endl;
    // std::cout << "msg.header.stamp.nsec: " << msg.header.stamp.nsec << std::endl;
    // std::cout << "msg.header.stamp.sec: " << msg.header.stamp.sec << std::endl;
    // std::cout << "time taken: " << current_time - previous_time << std::endl;

    if (!initialized)
    {
        std::cout << "Waiting to be initialised... " << std::endl;
        initialized = 1;
        std::cout << "Initialisation Done! " << std::endl;
    }
    else
    {
        duration = current_time.sec - previous_time.sec + (current_time.nsec - previous_time.nsec) / 1000000000.0;
        // std::cout << "duration: " << duration << std::endl;
        velocity_x = (current_x - previous_x) / duration;
        velocity_y = (current_y - previous_y) / duration;
        // std::cout << "velocity_x: " << velocity_x << std::endl;
        // std::cout << "velocity_y: " << velocity_y << std::endl;

        // fill in message vel_msg to publish
        vel_msg.linear.x = velocity_x;
        vel_msg.linear.y = velocity_y;
        vel_pub.publish(vel_msg);
    }
    previous_x = current_x;
    previous_y = current_y;
    previous_time = current_time;
}

void Velocity_Converter::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // std::cout << "imageCalback" << std::endl;
    // convert image type from sensor_msgs::Image (ROS) to cv::Mat (OpenCV)
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // for display
    image.copyTo(display_image);
    // draw x, y velocity values onto the image
    char str_x[100];
    if (velocity_x > 0) // put '+' sign for positive values
    { 
        sprintf(str_x, "Linear velocity x: +%f", velocity_x);
    }
    else
    {
        sprintf(str_x, "Linear velocity x: %f", velocity_x);
    }
    putText(display_image, str_x, cv::Point2f(20, 50), 1, 3, cv::Scalar(0, 0, 255, 255), 5);
    char str_y[100];
    if (velocity_y > 0)
    {
        sprintf(str_y, "Linear velocity y: +%f", velocity_y);
    }
    else
    {
        sprintf(str_y, "Linear velocity y: %f", velocity_y);
    }
    putText(display_image, str_y, cv::Point2f(20, 100), 1, 3, cv::Scalar(0, 0, 255, 255), 5);
    // std::cout << "velocity x: " << velocity_x << std::endl;
    // std::cout << "velocity y: " << velocity_y << std::endl;
    // draw arrow to show velocity direction and magnitude
    cv::Point end_point;
    end_point.x = 640 + (int)(velocity_x * 500.0);
    end_point.y = 512 - (int)(velocity_y * 500.0);
    drawArrow(display_image, cv::Point(640, 512), end_point, cv::Scalar(0, 0, 255, 255), 30, 20); //arrow from the center of image (640, 512)

    // display the image with value and numbers
    imshow("Velocity Estimation", display_image); // Show our image inside it.

    // convert image back from cv::Mat to ros format
    cv_bridge::CvImage arrow_msg;
    arrow_msg.header = msg->header; // Same timestamp and tf frame as input image
    arrow_msg.encoding = "bgr8";
    arrow_msg.image = display_image; // Your cv::Mat

    // publish display image
    arrow_image_pub.publish(arrow_msg.toImageMsg());

    // // write display images in folder
    // const std::string NAME_image = "/home/zhonghuan/pics/";
    // static int x = 1;
    // std::string s;
    // std::ostringstream temp;
    // temp << x;
    // s = temp.str();
    // imwrite(NAME_image + s + ".jpg", display_image);
    // x++;
    // // output videos
    // output_video.write(display_image);
    // output_video << display_image;
    cv::waitKey(1); // 0 means press key to continue, 1 means wait for 1ms
}