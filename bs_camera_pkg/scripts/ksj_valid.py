#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

import time


from ctypes import *


def ksj_test():
    libKsj = cdll.LoadLibrary('libksjapi.so')
    libKsj.KSJ_Init()
    camCount = libKsj.KSJ_DeviceGetCount()
    print(f"found {camCount} KSJ cams!")
    

    for cam_idx in range(camCount):


        usDeviceType = c_int()
        nSerials = c_int()
        usFirmwareVersion = c_int()
        libKsj.KSJ_DeviceGetInformation(cam_idx, byref(usDeviceType), byref(nSerials), byref(usFirmwareVersion))
        # libKsj.KSJ_DeviceGetInformation(cam_idx, usDeviceType, nSerials, usFirmwareVersion)
        print(f"[Cam-{cam_idx}] usDeviceType={usDeviceType} nSerials={nSerials} usFirmwareVersion={usFirmwareVersion}")



        if nSerials.value == 10:
            print(f"long focal cam_idx = {cam_idx}")


    
    # cam_idx = 1
    # libKsj.KSJ_DeviceGetInformation(cam_idx, byref(usDeviceType), byref(nSerials), byref(usFirmwareVersion))
    # print(f"[Cam-{cam_idx}] usDeviceType={usDeviceType} nSerials={nSerials} usFirmwareVersion={usFirmwareVersion}")
    

    # libKsj.KSJ_SetSerials(cam_idx, 20)

    
    # nWidth = c_int()
    # nHeight = c_int()
    # nBitCount = c_int()
    # libKsj.KSJ_CaptureGetSizeEx(cam_idx, byref(nWidth), byref(nHeight), byref(nBitCount))
    # print("width = %d" % (nWidth.value))
    # print("height = %d" % (nHeight.value))
    # print("bitcount = %d" % (nBitCount.value))


    # DeviceGetInformation 

 
if __name__ == "__main__":
    ksj_test()

