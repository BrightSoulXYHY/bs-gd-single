#!/usr/bin/python2
# -*- coding: utf-8 -*-

#publish images
import rospy
from sensor_msgs.msg import Image
import gxipy as gx
from cv_bridge import CvBridge,CvBridgeError
import cv2
import sys

bridge=CvBridge()

device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
    sys.exit(1)
'''
每个相机的sn都是独特的，在相机下面写着，这样就可以利用这个分开长短焦相机
sn作为launch文件中外传入的参数
'''
sn_full_name = rospy.search_param('sn')
param = rospy.get_param(sn_full_name)
for key in param:  #反证只有一个键，虽说跟node有关
    sn = param.get(key).get('sn')
cam = device_manager.open_device_by_sn(sn)
#set parameters
cam.TriggerMode.set(gx.GxSwitchEntry.OFF)#set continuous acquisition
cam.ExposureTime.set(10000)  #set exposure time


cam.stream_on()

def talker():
    pub = rospy.Publisher('image_dh', Image, queue_size=1)
    rospy.init_node('publisher', anonymous=True)
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

