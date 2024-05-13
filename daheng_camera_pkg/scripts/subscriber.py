#!/usr/bin/python3
import os
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
  
# import bs_img

def callback(imgmsg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    #  solve realtive pose after acquiring the grayscale image
    # cameraMatrix = bs_img.bs_cfg_real_1m.camK_lf
    # distortedcoeffs = bs_img.bs_cfg_real_1m.distortion_coefffs
    # Rwt = bs_img.bs_cfg_real_1m.R_wt
    # plane_real_= bs_img.bs_cfg_real_1m.plane_real_ptL
    cv2.imshow('1',img_gray)
    cv2.waitKey(0)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("/webcam", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()
