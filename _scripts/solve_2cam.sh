#! /bin/bash

gnome-terminal -- bash -c "cd ~/bs-gd-single; source devel/setup.bash; roslaunch daheng_camera_pkg daheng_2cam.launch; exec bash"
sleep 5s

gnome-terminal -- bash -c "cd ~/bs-gd-single; source devel/setup.bash; roslaunch bs_multi_solver bs_solver_2cam.launch; exec bash"
