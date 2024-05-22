#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

# import sophuspy as sp

from bs_img.bs_img_airsim import *
from bs_img.bs_cfg_airsim_10m import *
from bs_imu.bs_imu import *


def find_matched_pts(last_pts,cnt_pts,max_err = 300):
    # 靶标中有5个点
    match_pt_idxs = [None]*5
    m = len(last_pts)
    n = len(cnt_pts)
    #is last/cnt_pts empty?
    if m == 0 or n == 0:
        return match_pt_idxs

    match_mat = np.ones(shape=(m,n))
    for i in range(m):
        for j in range(n):
            match_mat[i,j] = np.linalg.norm(last_pts[i] - cnt_pts[j]) 

    # 更新点数少于靶标点数，则从靶标点数中寻找匹配
    if m > n:
        min_idxs = np.argmin(match_mat,axis=0)
        for j,i in enumerate(min_idxs):
            if match_mat[i,j] < max_err:
                match_pt_idxs[i] = j
    # 否则从靶标中寻找匹配
    else:
        min_idxs = np.argmin(match_mat,axis=1)
        for i,j in enumerate(min_idxs):
            if match_mat[i,j] < max_err:
                match_pt_idxs[i] = j
    return match_pt_idxs



# def calc_pt_n(plane_real_pt,SE3_be_bf,SE3_cam_be,camK):
#     return np.dot(camK,(SE3_cam_be*SE3_be_bf)*plane_real_pt)


# def calc_M2(plane_real_pt,SE3_be_bf):
#     '''扰动模型求导'''
#     pt_be = SE3_be_bf*plane_real_pt
#     return np.vstack([
#         np.hstack([np.identity(3),vec3d_to_ssmatrix(-pt_be)]),
#         np.zeros(6)
#     ])


# def calc_M1(pt_n,SE3_cam_be,camK):
#     M1 = np.zeros(shape=(4,4))
    
#     Xn,Yn,Zn = pt_n
#     mat_KT = np.dot(camK,SE3_cam_be.matrix()[:3,:3])
#     M1[0,:3] = -(mat_KT[0]*Zn - Xn*mat_KT[2])/(Zn*Zn)
#     M1[1,:3] = -(mat_KT[1]*Zn - Yn*mat_KT[2])/(Zn*Zn)
#     return M1


# # cam_data_dicts = {
# #     "cam_K30":{
# #         "camK":cam_K30,
# #         "last_pts":last_pt_k30,
# #         "cnt_pts":cnt_gt_pts_k30,
# #         "SE3_cam_be":SE3_k30_be,
# #         "wi":9/cam_K30[0][0],
# #     },
# #     "cam_K60":{
# #         "camK":cam_K60,
# #         "last_pts":last_pt_k60,
# #         "cnt_pts":cnt_gt_pts_k60,
# #         "SE3_cam_be":SE3_k60_be,
# #         "wi":1/cam_K60[0][0],
# #     },
# # }

# def solve_multi_cam_se3(cam_data_dicts,SE3_be_bf):
#     SE3_result = SE3_be_bf
#     for iter_num in range(10000):
#         H = np.zeros((6,6))
#         b = np.zeros(6)
#         err_norm = []
#         for cam_data_dict in cam_data_dicts.values():
#             camK = cam_data_dict["camK"]
#             last_pts = cam_data_dict["last_pts"]
#             cnt_pts = cam_data_dict["cnt_pts"]
#             SE3_cam_be = cam_data_dict["SE3_cam_be"]
#             wi = cam_data_dict["wi"]
            
#             match_pt_idxs = find_matched_pts(last_pts,cnt_pts)
#             for idx, match_pt_idx in enumerate(match_pt_idxs):
#                 if match_pt_idx is None:
#                     continue
                
#                 plane_real_pt = plane_real_ptL[idx]
#                 plane_obs_pt = cnt_pts[match_pt_idx]
                
#                 pt_n = calc_pt_n(plane_real_pt,SE3_result,SE3_cam_be,camK)
#                 plane_proj_pt = pt_n[:2]/pt_n[2]
                
                
#                 e = plane_obs_pt - plane_proj_pt
#                 M1 = calc_M1(pt_n,SE3_cam_be,camK)
#                 M2 = calc_M2(plane_real_pt,SE3_result)
#                 J = np.dot(M1,M2)[:2]

#                 # H +=  np.dot(J.T,J)
#                 # b += -np.dot(J.T,e)  
#                 H +=  wi*np.dot(J.T,J)
#                 b += -wi*np.dot(J.T,e)  
#                 err_norm.append(np.linalg.norm(e))
#         inv_H = np.linalg.pinv(H)
#         dx = np.dot(inv_H,b)
#         SE3_result = sp.SE3.exp(dx)*SE3_result


        
#         # if not iter_num % 10:
#         #     print(f"[{iter_num}] dx={np.linalg.norm(dx):.6f}  err={np.average(err_norm):.6f} t_vec={SE3_result.translation()}")


#         if np.linalg.norm(dx) < 1e-6:
#             # print(f"break in {iter_num}")
#             # print(f"[{iter_num}] err={np.average(err_norm):.6f} ")
#             break
#     return SE3_result
