#!/usr/bin/python3
# -*- coding: utf-8 -*-
import itertools

import cv2
import numpy as np

from bs_imu.bs_imu import rot_vec_to_matrix

def vec_to_enu(t_vec,r_vec,R_wt):
    # PnP解算出来的R是靶标坐标系到相机系中的坐标
    # 逆变换求相机系的原点在靶标坐标系中的位置
    # 然后再从靶标坐标系变换到世界系
    R_ct = rot_vec_to_matrix(r_vec)
    return -np.dot(R_wt,R_ct.T.dot(t_vec))


def solve_plane_pt(plane_px_ptL,plane_real_ptL,cam_K):
    '''
    传入特征点列表，通过飞机特征点解算相对位姿
    '''
    result_dict = {
        "rpe": np.nan,
        "r_vec": np.array([np.nan]*3),
        "t_vec": np.array([np.nan]*3),
        "plane_px_calcL": np.array([]),
        "img_valid": False,
    }

    pt_num = len(plane_px_ptL)
    # 特征点不足以解算
    if pt_num < 5:
        return result_dict

    data_dictL = []
    # 从左至右排序
    plane_px_ptL_sorted = sorted(plane_px_ptL,key=lambda x:x[0])

    # 特征点大于5个以上
    for combineL in itertools.combinations(range(len(plane_px_ptL)),5):
        plane_px_calcL = np.array([
            plane_px_ptL_sorted[i]
            for i in combineL
        ])

        retval,r_vec,t_vec = cv2.solvePnP(plane_real_ptL, plane_px_calcL, cam_K, distCoeffs=None, flags=cv2.SOLVEPNP_SQPNP)
        # 计算出的t_vec是罗德里格斯参数
        # 计算重投影误差
        reproj_errL = []
        for plane_real_pt,plane_px_calc in zip(plane_real_ptL,plane_px_calcL):
            plane_proj_pt,_ = cv2.projectPoints(plane_real_pt, r_vec, t_vec, cam_K, distCoeffs=None)
            plane_px_err = plane_proj_pt.flatten() - plane_px_calc
            reproj_errL.append(np.linalg.norm(plane_px_err))
        reproj_err = np.average(reproj_errL)
        data_dict = {
            "rpe": reproj_err,
            "r_vec": r_vec.flatten(),
            "t_vec": t_vec.flatten(),
            "plane_px_calcL": plane_px_calcL,
        }
        data_dictL.append(data_dict)

    
    # 取重投影误差最小的
    data_dictL_dorted = sorted(data_dictL,key=lambda x: x["rpe"])
    data_dict = data_dictL_dorted[0]
    # # 重投影误差过大
    if data_dict["rpe"] < 10:
        result_dict = data_dict
        result_dict["img_valid"] = True

    return result_dict