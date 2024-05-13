#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError


import os
import cv2
import numpy as np

import time


from ctypes import *




class KSJ_Node():
    def __init__(self):
        self.img_bridge = CvBridge()
        
        self.lf_cam_info = {
            "img_pub": rospy.Publisher("/image_ksj_lf", ImageMsg, queue_size=1),
            "exposure_time": float(rospy.get_param("~lf_exposure_time")),

        }
        self.sf_cam_info = {
            "img_pub": rospy.Publisher("/image_ksj_sf", ImageMsg, queue_size=1),
            "exposure_time": float(rospy.get_param("~sf_exposure_time")),

        }

        self.ksj_lib = cdll.LoadLibrary('libksjapi.so')
        
        camCount = 0
        cam_init_cnt = 0
        while camCount != 2:
            cam_init_cnt += 1
            self.ksj_lib.KSJ_UnInit()
            self.ksj_lib.KSJ_Init()
            camCount = self.ksj_lib.KSJ_DeviceGetCount()
            print("found {} KSJ cams!".format(camCount))
            time.sleep(0.5)

            if cam_init_cnt > 5:
                os._exit(1)

        
        #self.ksj_lib.KSJ_LogSet(True,"/home/brightsoul/bs_ws/log")
        #self.ksj_lib.KSJ_LogSetEx(True,"/home/brightsoul/bs_ws/log",9)
        self.ksj_lib.KSJ_LogSet(True,"ksj_log")
		
		
        usDeviceType = c_int()
        nSerials = c_int()
        usFirmwareVersion = c_int()



        self.ksj_lib.KSJ_ExposureTimeSet.argtypes = (c_int, c_float)

        for cam_idx in range(camCount):
            self.ksj_lib.KSJ_DeviceGetInformation(cam_idx, byref(usDeviceType), byref(nSerials), byref(usFirmwareVersion))
            print("Cam:{:02d} usDeviceType={} nSerials={} usFirmwareVersion={}".format(
                    cam_idx,usDeviceType.value,nSerials.value,usFirmwareVersion.value
            ))

            # 长焦相机对应参数
            if nSerials.value == 10:
                print("long focal cam_idx = {}".format(cam_idx))
                self.init_ksj_cam(self.lf_cam_info,cam_idx)
                print("long focal init done")


            
            if nSerials.value == 20:
                print("long focal cam_idx = {}".format(cam_idx))
                self.init_ksj_cam(self.sf_cam_info,cam_idx)
                print("short focal init done")
    
    def init_ksj_cam(self,cam_info,cam_idx):
        nWidth = c_int()
        nHeight = c_int()
        nBitCount = c_int()
        
        self.ksj_lib.KSJ_CaptureGetSizeEx(cam_idx, byref(nWidth), byref(nHeight), byref(nBitCount))
        print("Cam:{:02d} nWidth={} nHeight={} nBitCount={}".format(
            cam_idx,nWidth.value,nHeight.value,nBitCount.value
        ))

        cam_info["cam_idx"] = cam_idx 
        cam_info["width"]   = nWidth.value 
        cam_info["height"]  = nHeight.value
        cam_info["bit_count"]  = nBitCount.value 
        cam_info["buffer_size"] = nWidth.value * nHeight.value * nBitCount.value / 8 
        cam_info["pRawData"] = create_string_buffer(int(cam_info["buffer_size"]))
        
        # 设置为软触发
        self.ksj_lib.KSJ_TriggerModeSet(cam_idx, 2)
        # 设置一下曝光时间，单位ms
        self.ksj_lib.KSJ_ExposureTimeSet(cam_idx, cam_info["exposure_time"])


    def get_img(self,cam_info):
        cam_idx = cam_info["cam_idx"]
        pRawData = cam_info["pRawData"]
        width = cam_info["width"]
        height = cam_info["height"]
        bit_count = cam_info["bit_count"]

        retValue = self.ksj_lib.KSJ_CaptureRgbData(cam_idx, pRawData)
        cam_info["image"] = np.frombuffer(pRawData, np.uint8).reshape(height, width, int(bit_count/8))
        return retValue

    def pub_img(self,cam_info,ros_time):
        img_msg = self.img_bridge.cv2_to_imgmsg(cam_info["image"], "passthrough")
        img_msg.header.stamp = ros_time
        cam_info["img_pub"].publish(img_msg)
        
    def run(self):
        img_cnt = 0
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            lf_retValue = self.get_img(self.lf_cam_info)
            sf_retValue = self.get_img(self.sf_cam_info)

            if lf_retValue == 1 and sf_retValue == 1:
                print("capture error with count {:05d}".format(img_cnt))
                print("lf_retValue = {}, sf_retValue = {}".format(lf_retValue,sf_retValue))
                break
            img_cnt += 1
            ros_time = rospy.Time.now()
            self.pub_img(self.lf_cam_info,ros_time)
            self.pub_img(self.sf_cam_info,ros_time)
            scale = 0.75
            # cv2.imshow("lf_cam_info", cv2.resize(self.lf_cam_info["image"],None,fx=scale,fy=scale))
            # cv2.imshow("sf_cam_info", cv2.resize(self.sf_cam_info["image"],None,fx=scale,fy=scale))
            # cv2.waitKey(1)
            rate.sleep()

    

    
 
if __name__ == "__main__":
    rospy.init_node('ksj_cam_node', anonymous=True)
    node = KSJ_Node()
    node.run()

