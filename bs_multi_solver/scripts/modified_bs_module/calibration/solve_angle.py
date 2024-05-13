#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
//AC：标定杆种AC的距离，单位为mm
//BC：标定杆种BC的距离，单位为mm
//true_point3D：三维重建出来的点
//angle：返回的角度
//point3DA：返回的只包含A点的坐标
'''
# 使用角度和A点获取另外两个点B、C的坐标

import numpy as np
from typing import List, Tuple
from math import acos, sin, cos

def getAngle(AC: float, BC: float, true_point3D: List[List[float]]) -> Tuple[List[List[float]], List[List[float]]]:
    angle = []
    point3DA = []
    d1 = AC
    d2 = BC
 
    i = 0
    while i < len(true_point3D):
    
        A = np.array(true_point3D[i])
        B = np.array(true_point3D[i + 1])
        C = np.array(true_point3D[i + 2])
        
        nj = (C - A) / np.linalg.norm(C - A)  #直线方向向量

        cosphi = nj[2]
        phi = acos(cosphi)

        costheta = nj[0] / sin(phi)
        theta = acos(costheta)

        angle_temp = [phi, theta]
        angle.append(angle_temp)

        i += 3
    
    erroB = 0.0
    erroC = 0.0
    true_point3D_A = []
    i = 0
    j = 0
   
    while i < len(true_point3D):
        A = np.array(true_point3D[i])
        B = np.array(true_point3D[i + 1])
        C = np.array(true_point3D[i + 2])

        true_point3D_A.append(A)

        phi = angle[j][0]
        theta = angle[j][1]

        nj = np.array([sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)])

        b = A + (d1 - d2) * nj
        c = A + d1 * nj

        erroB += np.linalg.norm(B - b)
        erroC += np.linalg.norm(C - c)

        i += 3
        j += 1

    point3DA = true_point3D_A
    average_erroB = erroB / (len(true_point3D) // 3)
    average_erroC = erroC / (len(true_point3D) // 3)
    
    # print("用角度表示B误差：", average_erroB)
    # print("用角度表示C误差：", average_erroC)
    # print()
 

    return angle, point3DA
