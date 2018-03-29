#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

## Previous clean-up
rm -rf /root/src/Firmware/Tools/sitl_gazebo/models/f550_amazing-tmp-*
rm -f /root/src/Firmware/posix-configs/SITL/init/lpe/f550_amazing-tmp-*
rm -f /root/src/Firmware/launch/posix_sitl.launch

# world setup #
cp /simulation/lpe/f550_amazing /root/src/Firmware/posix-configs/SITL/init/lpe/f550_amazing
cp /simulation/worlds/timpa_airfield.world /root/src/Firmware/Tools/sitl_gazebo/worlds/timpa_airfield.world
cp -r /simulation/models/f550_amazing /root/src/Firmware/Tools/sitl_gazebo/models/f550_amazing
cp -r /simulation/models/f550_amazing_dead_blue /root/src/Firmware/Tools/sitl_gazebo/models/f550_amazing_dead_blue
cp -r /simulation/models/landing_strip /root/src/Firmware/Tools/sitl_gazebo/models/landing_strip
cp -r /simulation/simulation_node /root/catkin_ws/src/

cd ~/catkin_ws; catkin build
source ~/catkin_ws/devel/setup.bash


roslaunch simulation_control posix_sitl_f550.launch &> /dev/null &
sleep 10
roslaunch simulation_control px4.launch fcu_url:="udp://:14550@127.0.0.1:14557" &> /dev/null &
roslaunch simulation_control simulation_control.launch> &> /dev/null &



rosrun web_video_server web_video_server _port:=80 _server_threads:=100 &> /dev/null &


sleep 3000
