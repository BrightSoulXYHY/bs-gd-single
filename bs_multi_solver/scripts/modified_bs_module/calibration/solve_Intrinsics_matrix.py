#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
//Img1_Width:相机一拍摄图片的宽
//Img1_Height:相机一拍摄图片的高
//Img2_Width:相机二拍摄图片的宽
//Img2_Height:相机二拍摄图片的高
//FundamentalMatrix:基础矩阵
//K1:返回的相机一的内参
//K2:返回的相机二的内参
'''

# 这个标定原理假定相机光点位于像面中心，内参里面都是拿w/2,h/2代替，且假定fx和fy相等

import numpy as np
from numpy.linalg import svd, norm

def getIntrinsics(Img1_Width, Img1_Height, Img2_Width, Img2_Height, FundamentalMatrix):
    u1 = Img1_Width / 2
    v1 = Img1_Height / 2
    P1 = np.array([u1, v1, 1])

    u2 = Img2_Width / 2
    v2 = Img2_Height / 2
    P2 = np.array([u2, v2, 1])

    # 计算极点e1
    _, _, V1 = svd(FundamentalMatrix)
    e1 = V1[-1] / V1[-1, -1]
    E1 = np.array([[0, -e1[2], e1[1]],
                   [e1[2], 0, -e1[0]],
                   [-e1[1], e1[0], 0]])

    # 计算极点e2
    Ft = FundamentalMatrix.transpose()
    _, _, V2 = svd(Ft)
    e2 = V2[-1] / V2[-1, -1]
    E2 = np.array([[0, -e2[2], e2[1]],
                   [e2[2], 0, -e2[0]],
                   [-e2[1], e2[0], 0]])

    I = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 0]])

    f1 = -np.sqrt(norm(P2 @ E2 @ I @ FundamentalMatrix @ P1) / norm(P2 @ E2 @ I @ FundamentalMatrix @ I @ FundamentalMatrix @ P2))
    f2 = -np.sqrt(norm(P1 @ E1 @ I @ FundamentalMatrix.transpose() @ P2) / norm(P1 @ E1 @ I @ FundamentalMatrix.transpose() @ I @ FundamentalMatrix @ P1))

    K1_temp = np.array([[f1, 0, u1],
                        [0, f1, v1],
                        [0, 0, 1]])

    K2_temp = np.array([[f2, 0, u2],
                        [0, f2, v2],
                        [0, 0, 1]])

    return K1_temp, K2_temp
