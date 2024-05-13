#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
from bs_img import bs_img_real
from bs_img import bs_img_base
from bs_img.gauss_fit import *
from bs_img.bs_cfg_real_1m import *
import bs_imu
import solve

data_total=np.array([])
path="E:\\GalaxyPicture\\data\\"
#直接调用写好的函数，阈值+联通区域分析
dis = 30
N = 50
camera_matrix = camK_lf
distortion_coeffs = distortion_coeffs
for i in range(0,N):
    img_path = path+str(dis)+"+"+str(i)+".jpg"
    # img_draw=Image.open(img_path)
    img = cv2.imread(img_path,cv2.IMREAD_UNCHANGED)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.undistort(img_gray, camera_matrix, distortion_coeffs)  #去除相机成像畸变

    #求解位置  内参在bs_cfg_real_1m.py里面，记得改
    gt_pts, aera_list = bs_img_real.img_to_pts(img_gray,min_area=5, max_area=1111111111, threshold=200)

    #估计结果  结果不好
    result_dict = bs_img_base.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_lf)
    t_vec = result_dict["t_vec"]
    r_vec = result_dict["r_vec"]
    q_vec = bs_imu.bs_imu.rot_vec_to_quat(r_vec)  #罗德里格斯参数转四元数
    rpy_vec = bs_imu.bs_imu.quat_to_rpy(q_vec)   #四元数转欧拉角
    
    data_total=np.append(data_total,t_vec)
    data_total=np.append(data_total,rpy_vec)
    data_total=np.append(data_total,result_dict["rpe"])
    
#每一行顺序：三轴位置、罗德里格斯参数、重投影误差
data_total=data_total.reshape(N,7)
np.savetxt(path+'t_'+str(dis)+'m.txt',data_total,delimiter=',')




# #标出识别出的点
# draw = ImageDraw.Draw(img_draw)
# radius = 10
# print(len(gt_pts))
# for i in range(0,len(gt_pts)):
#     draw.ellipse((gt_pts[i][0] - radius, gt_pts[i][1] - radius, gt_pts[i][0] + radius, gt_pts[i][1] + radius), fill='black')
# plt.imshow(img_draw)
# plt.axis("off")
# plt.show()

