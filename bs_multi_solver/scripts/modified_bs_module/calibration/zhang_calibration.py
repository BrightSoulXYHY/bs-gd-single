#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
利用张正友标定法的原理及使用的平面红外靶标（五个点）标定相机内参

大靶标太亮了，换个小靶标试试？
'''

import numpy as np
import cv2
import sys, os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bs_img.bs_img_real import *


path = "E:\\GalaxyPicture\\exp4\\5+"

N = 50  # 图片总数
M = 5   # 用于标定的LED数目
img_width = 2448
img_height = 2048
cameraMatrix0 = np.array([[7000, 0, img_width/2], [0, 7000, img_height/2], [0, 0, 1]])
rvecs0 = np.array([0, 0, 0])
tvecs0 = np.array([0, 0, 5])
dist_coeffs0 = np.zeros(5)
plane_real_pts = [
    [-0.50, 0.00, 0],
    [-0.20, -0.10, 0],
    [0.00, 0.00, 0],
    [0.20, -0.10, 0],
    [0.50, 0.00, 0]
]

plane_real_pts_total = []
plane_img_pts_total = []

for i in range(0, N):
    img = cv2.imread(path + str(i) + ".jpg")
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    plane_img_pts, _ = img_to_pts(img_gray, min_area=5, max_area=44444444444, threshold=220)
    plane_img_pts_sorted = sorted(plane_img_pts, key=lambda x: x[0])
    plane_real_pts_total.append(plane_real_pts)
    plane_img_pts_total.append([[plane_img_pts_sorted_pt] for plane_img_pts_sorted_pt in plane_img_pts_sorted])


plane_img_pts_total = np.array(plane_img_pts_total,dtype=np.float32)
plane_real_pts_total = np.array(plane_real_pts_total,dtype=np.float32)

# 标定  感觉不稳定
retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    plane_real_pts_total, plane_img_pts_total, img_gray.shape[::-1], cameraMatrix0, dist_coeffs0
)


print("重投影误差：",retval)          
print("内参矩阵：",cameraMatrix)
print("畸变系数：",distCoeffs)
print("外参-旋转：", rvecs)
print("外参-平移：", tvecs)