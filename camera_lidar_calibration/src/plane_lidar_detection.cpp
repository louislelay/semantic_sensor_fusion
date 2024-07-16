#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Votre code de callback ici
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_corner_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/camera/corner_image", 10);

    ros::spin();
    return 0;
}
