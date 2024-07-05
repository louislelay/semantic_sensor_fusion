# Semantic Sensor Fusion for Unitree Go1

Welcome to the repository for the Semantic Sensor Fusion for Unitree Go1 project, developed by the Real World Robot Informatics Lab at the University of Tokyo. 

This project aims to enhance the robot's ability to associate its surroundings with objects, thereby imbuing them with meaning, and subsequently applying SLAM (Simultaneous Localization and Mapping) techniques.

This work is part of the research conducted by PhD student Jiaxu Wu.

![Research Topic of PhD Student Jiaxu Wu](medias/wu_jiaxu_research.png)

[Real World Robot Informatics Lab's Website](https://www.robot.t.u-tokyo.ac.jp/yamalab/index.html)

[Qi An's Website](https://www.robot.t.u-tokyo.ac.jp/anlab/)

## Code

The robot operates using ROS1 Kinetic. If you're not using Ubuntu 16, try to create a Docker image. There's one based on the OSRF ROS-Desktop-Full Kinetic environment. I'll publish a template of it soon.

To use my packages you need to install in addition to ROS Kinetic: 

```bash
sudo apt-get update && apt-get install -q -y --no-install-recommends \
	ros-kinetic-velodyne \
	ros-kinetic-realsense2-camera \
	ros-kinetic-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/velodyne.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin_make
```

## LiDAR VLP16 sensor

To configure the LiDAR, you have to declare the ip port, verify with wireshark what are yours and then run:

```bash
sudo ifconfig enp2s0f2 192.168.3.100
sudo route add 192.168.1.201 enp2s0f2
```

Using terminator, in the first terminal run: 

```bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

In a second one, run:

```bash
rosnode list
rostopic echo /velodyne_points
```

Stop it and then run (the arg is to put velodyne as a fixed frame):

```bash
rosrun rviz rviz -f velodyne
```

Add "cloudpoint2", and then select the topic "/velodyne_points". If you see the cloud point, it means everything is working correctly.

## RealSense Camera

The configuration is automatically done. To test it:

```bash
roslaunch realsense2_camera demo_pointcloud.launch
```

If you see the cloud point, it means everything is working correctly
