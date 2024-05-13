#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
//points1, points2:为1号与2号两台相机拍摄的标定杆上标定球的中心的像素点坐标。数据排列方式与求解基础矩阵的数据排列一样(就是一个东西）
//P1:相机一的投影矩阵
//P2:相机二的投影矩阵
//true_point3D:重建出来的三维点
//AverageReProjectError1:返回的相机一的平均重投影误差
//AverageReProjectError2:返回的相机二的平均重投影误差
//AverageDistanceAB:返回的AB的平均距离
//AverageDistanceBC:返回的BC的平均距离
//flag:是否打印每个点的数据
'''

import numpy as np
from typing import List
from typing import Tuple

def get_average_error(points1: List[np.ndarray], points2: List[np.ndarray],
                      P1: np.ndarray, P2: np.ndarray,
                      true_point3D: List[np.ndarray], flag: bool) -> Tuple[float, float, float, float]:
    average_reproject_error1 = 0.0
    average_reproject_error2 = 0.0
    average_distance_AB = 0.0
    average_distance_BC = 0.0
    
    camera1_everyReproErro=[]
    camera2_everyReproErro=[]
    
    for i in range(len(true_point3D)):
        tdpoint = np.array([true_point3D[i][0], true_point3D[i][1], true_point3D[i][2], 1])
        PPP1 = np.dot(P1, tdpoint)
        PPP2 = np.dot(P2, tdpoint)
        PPP1 /= PPP1[2]
        PPP2 /= PPP2[2]
        camera1_everyReproErro.append(np.linalg.norm(np.array([PPP1[0], PPP1[1]]) - points1[i]))
        camera2_everyReproErro.append(np.linalg.norm(np.array([PPP2[0], PPP2[1]]) - points2[i]))
        
        if flag:
            print("相机1重投影：")
            print(P1.dot(tdpoint))
            print("原始像素点：", points1[i])
            print()
            print("相机2重投影：")
            print(P2.dot(tdpoint))
            print("原始像素点：", points2[i])
            print()
            print("相机1差值：", np.linalg.norm(np.array([PPP1[0], PPP1[1]]) - points1[i]))
            print("相机2差值：", np.linalg.norm(np.array([PPP2[0], PPP2[1]]) - points2[i]))
        
        average_reproject_error1 += np.linalg.norm(np.array([PPP1[0], PPP1[1]]) - points1[i])
        average_reproject_error2 += np.linalg.norm(np.array([PPP2[0], PPP2[1]]) - points2[i])
    
    average_reproject_error1 /= len(points1)
    average_reproject_error2 /= len(points2)
    
    for i in range(0, len(true_point3D), 3):
        A = true_point3D[i]
        B = true_point3D[i + 1]
        C = true_point3D[i + 2]
        
        if flag:
            print("A到B的距离：", np.linalg.norm(B - A))
            print("B到C的距离：", np.linalg.norm(C - B))
        
        average_distance_AB += np.linalg.norm(B - A)
        average_distance_BC += np.linalg.norm(C - B)
    
    average_distance_AB /= len(true_point3D) // 3
    average_distance_BC /= len(true_point3D) // 3
    
    return average_reproject_error1, average_reproject_error2, average_distance_AB, average_distance_BC
