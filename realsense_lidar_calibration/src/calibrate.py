#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
import numpy as np
import tf
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraLidarFusion:
    def __init__(self):
        rospy.init_node('camera_lidar_fusion', anonymous=True)

        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        # Subscribers
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.lidar_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)

        # Synchronize topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        # Publisher
        self.pub = rospy.Publisher('/camera_lidar_fusion', PointCloud2, queue_size=10)

        # Camera intrinsic parameters (example values, you need to use your calibrated values)
        self.camera_matrix = np.array([[616.36529541, 0., 320.25881958],
                                       [0., 616.20294189, 240.0499115],
                                       [0., 0., 1.]])
        self.dist_coeffs = np.array([0., 0., 0., 0., 0.])

        # Extrinsic parameters between camera and LiDAR (example values)
        self.translation = np.array([0.1, 0.0, 0.0])  # Translation vector
        self.rotation = np.eye(3)  # Rotation matrix

    def callback(self, image_msg, lidar_msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

            # Transform LiDAR points to the camera frame
            points = self.transform_lidar_points(lidar_msg)

            # Project LiDAR points to the image plane
            image_points = self.project_points_to_image(points)

            # Overlay points on the image
            for pt in image_points:
                cv2.circle(cv_image, (int(pt[0]), int(pt[1])), 2, (0, 255, 0), -1)

            # Convert OpenCV image back to ROS Image message and publish
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except CvBridgeError as e:
            rospy.logerr(e)

    def transform_lidar_points(self, lidar_msg):
        # Convert PointCloud2 message to numpy array
        points_list = []
        for point in point_cloud2.read_points(lidar_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        points = np.array(points_list)

        # Apply transformation (extrinsic calibration)
        points = np.dot(self.rotation, points.T).T + self.translation
        return points

    def project_points_to_image(self, points):
        # Project 3D points to 2D image plane
        image_points, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.camera_matrix, self.dist_coeffs)
        return image_points.reshape(-1, 2)

if __name__ == '__main__':
    try:
        CameraLidarFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
