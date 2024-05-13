#!/usr/bin/python3
# -*- coding: utf-8 -*-

#publish images
import rospy
from sensor_msgs.msg import Image
import gxipy as gx
from cv_bridge import CvBridge,CvBridgeError
import cv2

bridge=CvBridge()

device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
    sys.exit(1)
str_sn = dev_info_list[0].get("sn")
cam = device_manager.open_device_by_sn(str_sn)
#set parameters
cam.TriggerMode.set(gx.GxSwitchEntry.OFF)#set continuous acquisition
cam.ExposureTime.set(1000)  #set exposure time


cam.stream_on()

def talker():
    pub = rospy.Publisher('webcam', Image, queue_size=1)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    cam.ExposureTime.set(10000)  #set exposure time
    while not rospy.is_shutdown():
        raw_image = cam.data_stream[0].get_image()
        rgb_image = raw_image.convert("RGB")
        if rgb_image is None:
            continue
        numpy_image = rgb_image.get_numpy_array()
        if numpy_image is None:
            continue
        msg=bridge.cv2_to_imgmsg(numpy_image,"bgr8")   #bgr image
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF==ord('q'):
            break
        if rospy.is_shutdown():
            cam.stream_off()
            cam.close_device()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

