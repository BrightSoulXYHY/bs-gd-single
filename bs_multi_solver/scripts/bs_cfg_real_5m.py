#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np

# 靶标坐标系，默认和相机系重合
# 点从左至右,单位m    x周指向右，y周指向下，原点在中间的LED
plane_real_ptL = np.array([
    [-2.425, -0.35, 1.87],
    [-0.75, -0.31, 0.18],
    [0, 0.12, 0.25],
    [0.750, -0.310, 0.180],
    [2.425, -0.35, 1.87]   
])
d_drogue = 0.4

# 世界系为ENU则有靶标系至世界系的转换矩阵
R_wt = np.array([
    [0,0,1],
    [-1,0,0],
    [0,-1,0],
])



#视场角约14.61
# # E:\\GalaxyPicture\\exp1
# camK_lf = np.array([
#     [7281.038109721694, 0.0000,  1222.263140039430],
#     [0.0000, 7300.026629124834,  1102.041992272973],
#     [0.0000,  0.0000,  1.0000],
# ])
# #E:\\GalaxyPicture\\exp2
# camK_lf = np.array([
#     [9432.431994466819, 0.0000,  1042.153536040823],
#     [0.0000, 9445.336276968366,  850.0072793998182],
#     [0.0000,  0.0000,  1.0000],
# ])

#exp4 = exp5, 镜头旋到底了，无穷远               
# 标定的时候重投影误差只有0.03
camK_sf = np.array(
   [[7763.53693, 0.00000000, 1221.51558],
    [000000000, 7896.75676, 1025.98376],
    [0.00000000, 0.00000000, 1.00000000]]
 )
# used in Gansu
# camK_sf = np.array(
#    [[7108.53693, 0.00000000, 1221.51558],
#     [000000000, 7230.75676, 1025.98376],
#     [0.00000000, 0.00000000, 1.00000000]]
#  )
distortion_coeffs_sf = np.array([
    [ 1.37369289e-01, -1.25314205e+01, -1.56390713e-03, -4.70760679e-02, 9.15447618e+02]
])         
camK_lf = np.array(
   [[7763.53693, 0.00000000, 1221.51558],
    [000000000, 7896.75676, 1025.98376],
    [0.00000000, 0.00000000, 1.00000000]]
 )
distortion_coeffs_lf = np.array([
    [ 1.37369289e-01, -1.25314205e+01, -1.56390713e-03, -4.70760679e-02, 9.15447618e+02]
])         



# 58 49 35

# # 视场角约110度

# 相机系到FLU机体系的旋转矩阵
R_bc = np.array([
    [0,0,1],
    [-1,0,0],
    [0,-1,0],
])
R_cb = R_bc.T
