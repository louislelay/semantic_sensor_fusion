import rospy
import yaml
import os
import tf
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
import message_filters
from cv_bridge import CvBridge
import cv2

# Function to load parameters from YAML file
def load_params():
    config_file = rospy.get_param("~config_file")
    if not os.path.isfile(config_file):
        rospy.logerr("Config file not found: {}".format(config_file))
        return None

    with open(config_file, 'r') as file:
        try:
            params = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            rospy.logerr("Error reading config file: {}".format(exc))
            return None

    return params

# Function to convert ROS PointCloud2 to PCL point cloud
def ros_to_pcl(ros_cloud):
    points_list = []

    for data in point_cloud2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)

    return pcl_data

# Function to apply voxel grid filtering
def voxel_grid_filter(pcl_data, leaf_size):
    voxel_filter = pcl_data.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return voxel_filter.filter()

# Function to transform point cloud based on calibration parameters
def transform_point_cloud(pcl_data, transform_matrix):
    points = np.array(pcl_data)
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_points = transform_matrix.dot(points_homogeneous.T).T
    transformed_pcl = pcl.PointCloud()
    transformed_pcl.from_array(transformed_points[:, :3].astype(np.float32))
    return transformed_pcl

# Callback function for synchronized data
def callback(lidar_msg, camera_msg):
    bridge = CvBridge()
    
    # Convert ROS Image to OpenCV image
    camera_image = bridge.imgmsg_to_cv2(camera_msg, desired_encoding="passthrough")

    # Convert ROS PointCloud2 to PCL
    lidar_pcl = ros_to_pcl(lidar_msg)

    # Load parameters from YAML file
    params = load_params()
    if params is None:
        rospy.logerr("Failed to load parameters.")
        return
    
    # Extract calibration parameters
    translation = params['extrinsics']['translation']
    rotation = params['extrinsics']['rotation']
    leaf_size = params['point_cloud_filtering']['voxel_leaf_size']
    
    # Create transformation matrix
    translation_vector = np.array([translation['x'], translation['y'], translation['z']])
    rotation_vector = np.array([rotation['roll'], rotation['pitch'], rotation['yaw']])
    transform_matrix = tf.transformations.compose_matrix(translate=translation_vector,
                                                         angles=rotation_vector)

    # Filter point cloud
    filtered_pcl = voxel_grid_filter(lidar_pcl, leaf_size)

    # Transform point cloud
    transformed_pcl = transform_point_cloud(filtered_pcl.to_array(), transform_matrix)

    # Visualization
    if params['visualization']['show_point_cloud']:
        pcl.visualization.CloudViewing().ShowMonochromeCloud(transformed_pcl)

    if params['visualization']['show_camera_image']:
        cv2.imshow("Camera Image", camera_image)
        cv2.waitKey(1)

def calibrate():
    rospy.init_node('realsense_lidar_calibration', anonymous=True)
    
    # Subscribers
    lidar_sub = message_filters.Subscriber('/lidar_points', PointCloud2)
    camera_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

    # Synchronize messages
    ts = message_filters.ApproximateTimeSynchronizer(
        [lidar_sub, camera_sub],
        rospy.get_param('~synchronization/queue_size', 10),
        rospy.get_param('~synchronization/slop', 0.1)
    )
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        calibrate()
    except rospy.ROSInterruptException:
        pass
