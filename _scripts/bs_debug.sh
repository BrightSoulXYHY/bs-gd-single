#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

RUN_DATE=`date +%Y%m%d_%H%M%S`



gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch daheng_camera_pkg daheng_camera.launch;exec bash"
sleep 5s


gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch bs_multi_solver bs_solver.launch;exec bash"
sleep 5s


