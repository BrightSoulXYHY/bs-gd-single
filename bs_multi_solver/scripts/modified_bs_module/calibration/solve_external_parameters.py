#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
//K1:相机一内参
//K2:相机二内参
//F:基础矩阵
//points1、points2:为1号与2号两台相机拍摄的标定杆上标定球的中心的像素点坐标。数据排列方式与求解基础矩阵的数据排列一样(就是一个东西）
//AC:标定杆A点与C点的真实距离,单位为mm
//R_out:返回的旋转矩阵
//t_out:返回的平移矩阵
'''


import numpy as np
from typing import List
from typing import Tuple

def getRt(K1: np.ndarray, K2: np.ndarray, F: np.ndarray,
          points1: List[np.ndarray], points2: List[np.ndarray],
          AC: float) -> Tuple[np.ndarray, np.ndarray]:
    E = np.dot(np.dot(K2.T, F), K1)
    _, _, V = np.linalg.svd(E)
    C = np.diag([1, 0, 0])  # Diagonal matrix with singular values
    S = np.dot(np.dot(V.T, E), np.linalg.inv(V))
    U, _, _ = np.linalg.svd(E)
    
    W = np.array([[0, -1, 0],
                  [1, 0, 0],
                  [0, 0, 1]])
    
    zze = np.array([[0, 0, 1]])
    
    R1 = np.dot(np.dot(U, W), V.T)
    t1 = np.dot(U, zze.T)
    
    R2 = np.dot(np.dot(U, W), V.T)
    t2 = -np.dot(U, zze.T)
    
    R3 = np.dot(np.dot(U, W.T), V.T)
    t3 = np.dot(U, zze.T)
    
    R4 = np.dot(np.dot(U, W.T), V.T)
    t4 = -np.dot(U, zze.T)
    
    P1 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0]])
    P1 = np.dot(K1, P1)
    
    R = None
    t = None
    
    for i, (R_temp, t_temp) in enumerate([(R1, t1), (R2, t2), (R3, t3), (R4, t4)]):
        P = np.array([[R_temp[0, 0], R_temp[0, 1], R_temp[0, 2], t_temp[0, 0]],
                      [R_temp[1, 0], R_temp[1, 1], R_temp[1, 2], t_temp[1, 0]],
                      [R_temp[2, 0], R_temp[2, 1], R_temp[2, 2], t_temp[2, 0]]])
        P = np.dot(K2, P)
        
        point3D = get3Dpoint(points1, points2, P1, P)
        flag = False
        for j in range(0, len(point3D), 3):
            if point3D[j + 2, 0] < 0:
                flag = True
                break
        if not flag:
            R = R_temp
            t = t_temp
            break
    
    P2 = np.array([[R[0, 0], R[0, 1], R[0, 2], t[0, 0]],
                   [R[1, 0], R[1, 1], R[1, 2], t[1, 0]],
                   [R[2, 0], R[2, 1], R[2, 2], t[2, 0]]])
    P2 = np.dot(K2, P2)
    
    point3D = []
    
    lambda_val = 0
    L_Li = 0
    
    point3D = get3Dpoint(points1, points2, P1, P2)
    
    for i in range(0, len(point3D), 3):
        A = point3D[i, :]
        C = point3D[i + 2, :]
        
        AC_ = np.linalg.norm(C - A)
        
        L_Li += AC / AC_
        
    lambda_val = L_Li / (len(points1) / 3)
    t = lambda_val * t
    
    return R, t


def get3Dpoint(input1: List[np.ndarray], input2: List[np.ndarray],
               P1: np.ndarray, P2: np.ndarray) -> np.ndarray:
    if len(input1) != len(input2):
        print("getFundamentalMatrix函数输入两组点的个数不相同")
        exit(0)
    
    projection3Dpoint = []
   
    for i in range(len(input1)):
        A = np.zeros((4, 4))
        
        for j in range(0, 4, 2):
       
            point = input1[i] if j == 0 else input2[i]
            P = P1 if j == 0 else P2

            A[j,:] = point[0] * P[2,:] - P[0,:]
            A[j + 1,:] = point[1] * P[2,:] - P[1,:]
       
        _, _, V = np.linalg.svd(A)
        X = V[-1] / V[-1, -1]
        projection3Dpoint.append(X[:-1])

    return np.array(projection3Dpoint)
