#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError


import cv2
import time



def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
 
 
def csi_publish():
    rospy.init_node('csi_cam_node')
    # 中心翻转180
    cap = cv2.VideoCapture(gstreamer_pipeline(framerate=24,flip_method=2), cv2.CAP_GSTREAMER)
    
    img_bridge = CvBridge() 
    img_pub = rospy.Publisher("/image_csi", ImageMsg, queue_size=1)


    while cap.isOpened():
        _, img = cap.read()
        img_msg = img_bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg)
        cv2.imshow("CSI", cv2.resize(img,None,fx=0.5,fy=0.5))
        cv2.waitKey(1)

        time.sleep(0.01)
 
    cap.release()
    
 
if __name__ == "__main__":
    csi_publish()
