#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

from modified_bs_module.bs_img.bs_img_base import *

# Airsim理想内存矩阵
def fov2f(fov,w=1280):
    return w/(2*np.tan(np.deg2rad(fov/2)))

def constructK(fov=120,w=1280,h=720):
    f = fov2f(fov,np.max([w,h]))
    return np.array([
        [f,0,w/2],
        [0,f,h/2],
        [0,0,1],
    ],dtype=float)

cam_K60 = constructK(fov=60)
cam_K30 = constructK(fov=30)


def color_red_proc(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    chH,chS,chV = cv2.split(hsv_img)
    chH[chH < 60] += 180
    hsv_img = cv2.merge([chH,chS,chV])
    # Airsim红
    low_L = [160, 115, 0]
    up_L  = [190, 255, 255]
    img_bin = cv2.inRange(hsv_img, np.array(low_L), np.array(up_L))
    return img_bin

def img_to_pts(color_img, min_area = 15):
    '''
        提取图片中的特征点
        面积小于5的pass
    '''
    color_bin = color_red_proc(color_img)
    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(color_bin, connectivity=4)
    
    gt_pts = []
    area_list = []
    for stat,centroid in zip(stats[1:],centroids[1:]):
        if stat[-1] < min_area:
            continue
        gt_pts.append(centroid)
        area_list.append(stat[-1])
    return gt_pts,area_list
