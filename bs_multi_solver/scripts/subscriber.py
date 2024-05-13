#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

from modified_bs_module.bs_img.bs_cfg_real_1m import * 
from modified_bs_module import solve 

def callback(imgmsg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #  solve realtive pose after acquiring the grayscale image
    #  phisical parameters     from bs_cfg_real_1m   
    cameraMatrix = camK_lf
    distortedcoeffs = distortion_coeffs
    Rwt = R_wt
    plane_real_pt= plane_real_ptL
    data_total = solve.solve(img_gray, camera_matrix=cameraMatrix, distortion_coeffs= distortedcoeffs, 
                    plane_real_ptL= plane_real_pt, R_wt= Rwt)
    print(data_total)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("webcam", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()
