#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


# 世界系中的坐标点
plane_real_ptL = np.array([
    [-4.85,-0.35,-0.5],
    [-1.50,-0.95,-3.65],
    [ 0.05,-0.25,-3.75],
    [ 1.60,-0.95,-3.65],
    [ 4.90,-0.35,-0.5],
])



# 相机系到机体系(FRD)的旋转矩阵
R_bc = np.array([
    [0,0,1],
    [1,0,0],
    [0,1,0],
])

R_cb = R_bc.T
