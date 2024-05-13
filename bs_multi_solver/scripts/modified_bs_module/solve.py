#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import itertools
import math

from bs_img import bs_img_base
from bs_img import bs_img_real
'''
解算位姿         
输入：灰度图+相机内参矩阵+畸变系数+靶标位置坐标+靶标系至世界系（ENU）的转换矩阵
输出：三轴位置+三个欧拉角（rad）+重投影误差
'''

# solve函数只考虑图像视场内只有整个飞机大靶标的情形
def solve(img_gray, camera_matrix, distortion_coeffs, plane_real_ptL, R_wt):
    img_gray = cv2.undistort(img_gray, camera_matrix, distortion_coeffs)  #去除相机成像畸变

    #求解位置  内参在bs_cfg_real_1m.py里面，记得改
    gt_pts, aera_list = bs_img_real.img_to_pts(img_gray,min_area=5, max_area=1111111111, threshold=200)

    #估计结果  结果不好
    result_dict = bs_img_base.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camera_matrix)
    t_vec = result_dict["t_vec"]
    r_vec = result_dict["r_vec"]

    data_total = []
    data_total = np.append(data_total,t_vec)
    data_total = np.append(data_total,r_vec)
    data_total = np.append(data_total,result_dict["rpe"])
        
    #每一行顺序：三轴位置、罗德里格斯参数、重投影误差
    data_total = data_total.reshape(-1,7)

    return data_total

# solve_sf  solve_lf  分别考虑短、长焦相机下位置的解算
def solve_lf(gt_pts, camera_matrix, plan_real_ptL):     #输入靶标点像素、实际坐标和相机内参，输出位姿估计结果
    result_dict = bs_img_base.solve_plane_pt(gt_pts, plan_real_ptL, camera_matrix)
    t_vec = result_dict["t_vec"]
    r_vec = result_dict["r_vec"]
    
    return t_vec, r_vec

def solve_sf(gt_pts, camera_matrix, ):    
    
    return 
# solve_two函数分别依靠飞机靶标和锥套靶标两部分进行位姿估计
# 都用PnP吧，感觉近了之后飞机可能成像不全，没法去判断怎样拟合出来的圆是对的
# 输入灰度图、相机参数、阈值、飞机靶标真实位置、锥套靶标真实位置
def solve_two(img_gray,camera_matrix, distortion_coeffs, threshold, plane_real_ptL_lf, plane_real_ptL_sf):
    # 结果初始化
    result_dict_lf = {              #长焦
        "rpe": np.nan,
        "r_vec": np.array([np.nan]*3),
        "t_vec": np.array([np.nan]*3),
        "plane_px_calcL": np.array([]),
        "img_valid": False,
    }
    result_dict_sf = {             #短焦
        "rpe": np.nan,
        "r_vec": np.array([np.nan]*3),
        "t_vec": np.array([np.nan]*3),
        "plane_px_calcL": np.array([]),
        "img_valid": False,
    }
    
    #对图像的处理
    img_gray = cv2.undistort(img_gray, camera_matrix, distortion_coeffs)  #去除相机成像畸变
    gt_pts, _ = bs_img_real.img_to_pts(img_gray,min_area=5, max_area=1111111111, threshold= threshold)
    
    # 飞机靶标有五个  锥套靶标有八个   暂定
    pts_num = len(gt_pts)
    plane_px_ptL_sorted = sorted(gt_pts,key=lambda x:x[0])
    if pts_num < 5:
        return result_dict_lf, result_dict_sf #特征点不足以解算
    else:
        '''
        提取飞机靶标 
        '''
        data_dictL_lf = []
        # 从左至右排序
        plane_px_ptL_sorted = sorted(gt_pts,key=lambda x:x[0])
        for combineL in itertools.combinations(range(len(plane_px_ptL_sorted)),5):
            plane_px_calcL = np.array([
                plane_px_ptL_sorted[i]
                for i in combineL
            ])
            retval,r_vec,t_vec = cv2.solvePnP(plane_real_ptL_lf, plane_px_calcL, camera_matrix, distCoeffs=None, flags=cv2.SOLVEPNP_SQPNP) #SQPnP至少三个点
            # 计算出的t_vec是罗德里格斯参数
            # 计算重投影误差
            reproj_errL = []
            for plane_real_pt,plane_px_calc in zip(plane_real_ptL_lf,plane_px_calcL):
                plane_proj_pt,_ = cv2.projectPoints(plane_real_pt, r_vec, t_vec, camera_matrix, distCoeffs=None)
                plane_px_err = plane_proj_pt.flatten() - plane_px_calc
                reproj_errL.append(np.linalg.norm(plane_px_err))
            reproj_err = np.average(reproj_errL)
            data_dict = {
                "rpe": reproj_err,
                "r_vec": r_vec.flatten(),
                "t_vec": t_vec.flatten(),
                "plane_px_calcL": plane_px_calcL,
            }
            data_dictL_lf.append(data_dict)
        # 取重投影误差最小的
        data_dictL_dorted_lf = sorted(data_dictL_lf,key=lambda x: x["rpe"])
        data_dict_lf = data_dictL_dorted_lf[0]
        # # 重投影误差过大
        if data_dict_lf["rpe"] < 10:
            result_dict_lf = data_dict_lf
            result_dict_lf["img_valid"] = True
            # 获取剩下的点用来拟合锥套
            set_lf = set(map(tuple, data_dict_lf["plane_px_calcL"]))
            set_all = set(map(tuple, np.array(plane_px_ptL_sorted)))
            set_sf = set_all - set_lf
            plane_px_sf = [np.array(item) for item in set_sf]  
        else:
            plane_px_sf = plane_px_ptL_sorted

        '''
        提取锥套靶标 
        '''
        pts_num_sf = len(plane_px_sf)
        if pts_num_sf >= 8:
            data_dictL_sf = []
            for combineL in itertools.combinations(range(len(plane_px_sf)),8):
                plane_px_calcL = np.array([
                    plane_px_sf[i]
                    for i in combineL
                ])
                # 对其换个顺序，顺时针，最左边的点为第一个，跟真值相同
                plane_px_calcL = sort_circle(plane_px_calcL)
                plane_px_calcL = np.array(plane_px_calcL).astype(np.float32)      #转换成solvePnP的类型

                retval,r_vec,t_vec = cv2.solvePnP(plane_real_ptL_sf, plane_px_calcL, camera_matrix, distCoeffs=None, flags=cv2.SOLVEPNP_SQPNP) #SQPnP至少三个点
                # 计算出的t_vec是罗德里格斯参数
                # 计算重投影误差
                reproj_errL = []
                for plane_real_pt,plane_px_calc in zip(plane_real_ptL_lf,plane_px_calcL):
                    plane_proj_pt,_ = cv2.projectPoints(plane_real_pt, r_vec, t_vec, camera_matrix, distCoeffs=None)
                    plane_px_err = plane_proj_pt.flatten() - plane_px_calc
                    reproj_errL.append(np.linalg.norm(plane_px_err))
                reproj_err = np.average(reproj_errL)
                data_dict = {
                    "rpe": reproj_err,
                    "r_vec": r_vec.flatten(),
                    "t_vec": t_vec.flatten(),
                    "plane_px_calcL": plane_px_calcL,
                }
                data_dictL_sf.append(data_dict)
            # 取重投影误差最小的
            data_dictL_dorted_sf = sorted(data_dictL_sf,key=lambda x: x["rpe"])
            data_dict_sf = data_dictL_dorted_sf[0]
            if data_dict_sf["rpe"] < 10:
                result_dict_sf = data_dict_sf
                result_dict_sf["img_valid"] = True
                
    return result_dict_lf, result_dict_sf

# sort_circle 用于对锥套上提取的点的排序  顺时针，最左边的点为第一个点
def sort_circle(points):
    # 计算中心点
    center_x = sum(x for x, _ in points) / len(points)
    center_y = sum(y for _, y in points) / len(points)
    center = (center_x, center_y)

    # 计算每个点相对于中心点的极角
    def get_polar_angle(point):
        x, y = point
        dx = x - center_x
        dy = y - center_y
        return math.atan2(dy, dx)          #值域(-pi,pi]
    # 根据极角对点进行排序
    sorted_points = sorted(points, key=get_polar_angle, reverse=True)
    # 为了防止排序意外的发生（本应该是pi多一点点，结果由于误差成了-pi多一点点了）
    if sorted_points[-1][0] < sorted_points[0][0]:
        sorted_points.insert(0, sorted_points.pop())

    return sorted_points