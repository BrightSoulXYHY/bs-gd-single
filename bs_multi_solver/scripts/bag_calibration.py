#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
根据飞机上的五个灯标定相机
读入rosbag保存的图片
'''
import numpy as np
import cv2
import sys, os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bs_img import bs_img_real 
from bs_img import bs_cfg_real_5m as realparams

import time
import rosbag
from cv_bridge import CvBridge, CvBridgeError


# 读入包
start_time = time.time()

file_pwd = os.path.dirname(os.path.abspath(__file__))

SAVE_IMG = False

bag_name = "20m"
input_bag = rosbag.Bag(f"{file_pwd}/../../../bag/{bag_name}.bag")
out_dir = f"{file_pwd}/../../../out/{bag_name}"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)

topicL = [
    "/image_dh_sf",
]
# 初始参数
img_width = 2448
img_height = 2048
cameraMatrix0 = np.array([[7000, 0, img_width/2], [0, 7000, img_height/2], [0, 0, 1]])
rvecs0 = np.array([0, 0, 0])
tvecs0 = np.array([0, 0, 5])
dist_coeffs0 = np.zeros(5)
plane_real_pts = np.array([
    [-2.425, -0.35, 0],
    [-0.75, -0.31, 0],
    [0, 0.12, 0],
    [0.750, -0.310, 0],
    [2.425, -0.35, 0]   
])
threshold = 50

# 过程中会用到的变量
sf_data_cnt = 0

gt_dict = {
    "LLA" : None,
    "vel" : None,
    "acc" : None,
    "quat" : None,
    "ang_vel" : None,
}

img_bridge = CvBridge()
# 存入LED的真值位置和图像位置
plane_real_pts_total = []
plane_img_pts_total = []

for topic, msg, t in input_bag.read_messages(topics=topicL):
    # 判断是否初始化完成
    if None in gt_dict.keys():
        continue

    img = img_bridge.imgmsg_to_cv2(msg, msg.encoding)
    img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    _, img_bin = cv2.threshold(img_gray, threshold, 0xff, cv2.THRESH_BINARY)
    gt_pts,_ = bs_img_real.img_to_pts(img_bin)
    result_dict = bs_img_real.solve_plane_pt(plane_px_ptL=gt_pts,plane_real_ptL=plane_real_pts,cam_K=realparams.camK_sf)
    
    # print(result_dict)
    plane_img_pts_sorted = sorted(result_dict["plane_px_calcL"], key=lambda x: x[0])
    plane_real_pts_total.append(plane_real_pts)

    plane_img_pts_total.append([plane_img_pts_sorted_pt for plane_img_pts_sorted_pt in plane_img_pts_sorted])
     
    # with open(f"{out_dir}/{sf_data_cnt:05d}_sf.json","w") as fp:
    #     json.dump(save_data,fp,indent=4)
    if SAVE_IMG:
        cv2.imwrite(f"{out_dir}/{sf_data_cnt:05d}_sf.png",img_gray)
    sf_data_cnt += 1

plane_img_pts_total = np.array(plane_img_pts_total,dtype=np.float32)
plane_real_pts_total = np.array(plane_real_pts_total,dtype=np.float32)

# 标定  感觉不稳定
retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    plane_real_pts_total, plane_img_pts_total, img_gray.shape[::-1], cameraMatrix0, dist_coeffs0
)


print("重投影误差：",retval)          
print("内参矩阵：",cameraMatrix)
print("畸变系数：",distCoeffs)
# print("外参-旋转：", rvecs)
# print("外参-平移：", tvecs)