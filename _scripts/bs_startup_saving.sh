#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

RUN_DATE=`date +%Y%m%d_%H%M%S`



gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
sleep 5s

gnome-terminal -x bash -c "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/KSJ/lib64;source ${WS_DIR}/devel/setup.bash;roslaunch bs_camera_pkg bs_ksj.launch;exec bash"
sleep 5s


gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosrun bs_camera_pkg bag_saver.py;exec bash"
sleep 5s




