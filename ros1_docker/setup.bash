# xauth
xauth merge /dot.Xauthority

# ROS
source /opt/ros/kinetic/setup.bash

# LiDAR setup
sudo ifconfig enp2s0f2 192.168.3.100
sudo route add 192.168.1.201 enp2s0f2
