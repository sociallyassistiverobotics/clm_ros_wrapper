#!/bin/bash

sleep 1s

echo "Running SAR perception"

source /home/sar/catkin_ws/devel/setup.bash
# publish own ip
export ROS_IP=192.168.2.5
# set the master ip
export ROS_MASTER_URI=http://192.168.2.3:11311

ros_master_ready=false
# rosnode_output=$(rosnode list 2>&1)

# if [[ $rosnode_output == *"ERROR"* ]]; then
#     ros_master_ready=false
# else
# 	ros_master_ready=true
# fi

# echo ${ros_master_ready}

until [ $ros_master_ready == true ]
do
	# echo ${ros_master_ready}
	# sleep 1s

	rosnode_output=$(rosnode list 2>&1)

	if [[ $rosnode_output == *"ERROR"* ]]; then
	    ros_master_ready=false
	    sleep 1s
	else
		ros_master_ready=true
	fi
	echo ${ros_master_ready}
done

echo "connected to ros master"
roslaunch clm_ros_wrapper clm_ros_wrapper.launch

# roslaunch clm_ros_wrapper clm_ros_wrapper_record.launch

# https://askubuntu.com/questions/187071/how-do-i-shut-down-or-reboot-from-a-terminal/187080
sleep 2s
/usr/bin/dbus-send --system --print-reply --dest="org.freedesktop.ConsoleKit" /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop
