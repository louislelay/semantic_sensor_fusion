#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class CameraLidarCalibration {
public:
    CameraLidarCalibration(ros::NodeHandle& nh)
        : it_(nh)
    {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &CameraLidarCalibration::imageCallback, this);
        image_pub_ = it_.advertise("/camera_lidar_calibration/output_image", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        // Detect aruco markers
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

            // Check if the markers form a calibration plane
            if (marker_ids.size() == 4) {
                for (size_t i = 0; i < marker_corners.size(); ++i) {
                    for (size_t j = 0; j < marker_corners[i].size(); ++j) {
                        cv::circle(image, marker_corners[i][j], 5, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        image_pub_.publish(out_msg.toImageMsg());
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_lidar_calibration");
    ros::NodeHandle nh;
    CameraLidarCalibration calibration(nh);
    ros::spin();
    return 0;
}
