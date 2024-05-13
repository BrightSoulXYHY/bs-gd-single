#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
用椭圆对边界进行拟合，返回椭圆的中心坐标
'''

import cv2
import numpy as np

def ellipse_fit(img_bin):
    # 查找轮廓
    contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # 椭圆拟合边界
    ellipses = []
    center = []
    for contour in contours:
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            ellipses.append(ellipse)
            center.append(list(ellipse[0]))
            
    return center




