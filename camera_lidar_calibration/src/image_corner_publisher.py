#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageCornerPublisher:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
		self.image_pub = rospy.Publisher("/camera/corner_image", Image, queue_size=10)
		
	def image_callback(self, msg):
		# Convert ROS image to OpenCV image
		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		
		# Convert the image to grayscale
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		
		# Define the size of the checkerboard
		checkerboard_size = (8, 5)  # Modify this according to your checkerboard size
		
		# Define the criteria for cornerSubPix
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		
		# Find the checkerboard corners
		ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
		
		# List to store the corner coordinates
		corner_coordinates = []

		# Colors for the circles
		blue_color = (255, 0, 0)  # Blue
		green_color = (0, 255, 0)  # Green

		if ret:
			# Refine the corner positions
			corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
			
			# Draw circles on the corners
			for i, corner in enumerate(corners):
				x, y = corner.ravel()
				x, y = int(x), int(y)  # Convert coordinates to integers
				if i == 0 or i == 7 or i == 39 or i == 32:  # Color the first four corners in blue
					cv_image = cv2.circle(cv_image, (x, y), 5, blue_color, -1)
					corner_coordinates.append((x, y))
				else:  # Color the rest in green
					cv_image = cv2.circle(cv_image, (x, y), 5, green_color, -1)
					
				
		# Convert the image back to ROS format and publish
		corner_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		self.image_pub.publish(corner_image_msg)

		
if __name__ == "__main__":
	rospy.init_node('image_corner_publisher', anonymous=True)
	ImageCornerPublisher()
	rospy.spin()


