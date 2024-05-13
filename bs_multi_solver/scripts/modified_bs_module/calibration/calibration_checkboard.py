#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import glob

w = 8
h = 5   # 5*8个内角点
size = (2448,2048)
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w * h, 3), np.float32)
objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
 
# 储存棋盘格角点的世界坐标和图像坐标对
obj_points = []  # 在世界坐标系中的三维点
img_points = []  # 在图像平面的二维点

images=glob.glob("E:\\FILES\\Rfly\\Circular marker\\Infrared_Circular_Marker_singleUAV\\callibration\\image_cali_phone\\image\\*.jpg")  #黑白棋盘的图片路径

for fname in images:
    img = cv2.imread(fname)
 
    # 修改图像尺寸，参数依次为：输出图像，尺寸，沿x轴，y轴的缩放系数，INTER_AREA在缩小图像时效果较好
    # img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # 转灰度
 
    # 找到棋盘格角点
    # 棋盘图像(8位灰度或彩色图像)  棋盘尺寸  存放角点的位置
    ret, corners = cv2.findChessboardCorners(gray, (w, h), None)
 
    # 角点精确检测
    # criteria:角点精准化迭代过程的终止条件(阈值)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    # 执行亚像素级角点检测
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
 
    obj_points.append(objp)
    img_points.append(corners2)


rpe, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
 
# 内参数矩阵
Camera_intrinsic = {"mtx": mtx,"dist": dist,}

print("重投影误差",rpe)
print("内参",Camera_intrinsic)
print("外参",tvecs,rvecs)
