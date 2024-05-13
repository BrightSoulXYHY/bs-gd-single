#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
//优化相机内参,外参,A点的三维坐标,以及欧拉角angle
//K 相机内参
//R 旋转矩阵(3*3)
//t 平移矩阵(3*1)
//true_point3D_A 只包含A点的三维坐标
//angle 欧拉角
//points points[0],表示第一台相机检测到的所有图片的一维标定杆的三个点的像素坐标
//AC 一维标定杆AC的长度,一维标定杆三个点按照ABC排列
//BC 一维标定杆BC的长度
'''

import numpy as np
from scipy.optimize import least_squares
import cv2

def bundle_adjustment(K, R, t, true_point3D_A, angle, points, AC, BC, max_num_iterations=200):
    d1 = AC
    d2 = BC

    # 相机数量
    camera_nums = len(K)

    def cost_function(parameters):
        # 提取内参和外参
        intrinsic = []
        extrinsics = []
        for i in range(camera_nums):
            K_temp = np.array([[parameters[i*4], 0, parameters[i*4+2]],
                               [0, parameters[i*4+1], parameters[i*4+3]],
                               [0, 0, 1]])
            intrinsic.append(K_temp)

            R_temp = cv2.Rodrigues(np.array([parameters[camera_nums*4+i*6],
                                             parameters[camera_nums*4+i*6+1],
                                             parameters[camera_nums*4+i*6+2]]))[0]
            t_temp = np.array([[parameters[camera_nums*4+i*6+3]],
                               [parameters[camera_nums*4+i*6+4]],
                               [parameters[camera_nums*4+i*6+5]]])
            extrinsics_temp = np.hstack((R_temp, t_temp))
            extrinsics.append(extrinsics_temp)

        # 重投影误差计算
        reprojection_errors = []
        for i in range(len(points)):
            P = true_point3D_A[i]
            for j in range(camera_nums):
                X = P[0]
                Y = P[1]
                Z = P[2]
                K_temp = intrinsic[j]
                R_temp = extrinsics[j][:, :3]
                t_temp = extrinsics[j][:, 3]

                # 针孔相机模型
                x = K_temp[0, 0] * (R_temp[0, 0] * X + R_temp[0, 1] * Y + R_temp[0, 2] * Z + t_temp[0]) / \
                    (R_temp[2, 0] * X + R_temp[2, 1] * Y + R_temp[2, 2] * Z + t_temp[2])
                y = K_temp[1, 1] * (R_temp[1, 0] * X + R_temp[1, 1] * Y + R_temp[1, 2] * Z + t_temp[1]) / \
                    (R_temp[2, 0] * X + R_temp[2, 1] * Y + R_temp[2, 2] * Z + t_temp[2])

                reprojection_errors.append(x - points[i][j][0])
                reprojection_errors.append(y - points[i][j][1])

        return reprojection_errors

    # 初始化优化参数
    parameters = []
    for i in range(camera_nums):
        K_temp = K[i]
        parameters.extend([K_temp[0, 0], K_temp[1, 1], K_temp[0, 2], K_temp[1, 2]])
    for i in range(camera_nums):
        R_temp = R[i]
        t_temp = t[i]
        R_vector_temp,J_temp = cv2.Rodrigues(R_temp)
        parameters.extend([R_vector_temp[0],R_vector_temp[1],R_vector_temp[2]])
        parameters.extend(t_temp)
    parameters=np.array(parameters,dtype=object).astype(float)

    # 执行优化
    result = least_squares(cost_function, parameters, max_nfev=max_num_iterations)

    # 提取优化结果
    optimized_parameters = result.x

    # 重建优化后的相机内参和外参
    intrinsic_optimized = []
    extrinsics_optimized = []
    for i in range(camera_nums):
        K_temp = np.array([[optimized_parameters[i*4], 0, optimized_parameters[i*4+2]],
                           [0, optimized_parameters[i*4+1], optimized_parameters[i*4+3]],
                           [0, 0, 1]])
        intrinsic_optimized.append(K_temp)

        R_temp = cv2.Rodrigues(np.array([optimized_parameters[camera_nums*4+i*6],
                                         optimized_parameters[camera_nums*4+i*6+1],
                                         optimized_parameters[camera_nums*4+i*6+2]]))[0]
        t_temp = np.array([[optimized_parameters[camera_nums*4+i*6+3]],
                           [optimized_parameters[camera_nums*4+i*6+4]],
                           [optimized_parameters[camera_nums*4+i*6+5]]])
        extrinsics_temp = np.hstack((R_temp, t_temp))
        extrinsics_optimized.append(extrinsics_temp)

    return intrinsic_optimized, extrinsics_optimized
