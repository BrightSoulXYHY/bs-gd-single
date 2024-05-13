#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.optimize import curve_fit
import cv2

#高斯曲面函数定义
def gaussian_surface(xy, a, mx, my, sx, sy):
    x = xy[0]
    y = xy[1]
    return a * np.exp(-((x - mx) ** 2 / (2 * sx ** 2) + (y - my) ** 2 / (2 * sy ** 2)))

# 使用高斯曲面拟合成像区域亮度，高斯曲面中心即为靶标中心
def fit_gaussian_surface(coordinates, brightness):

    # 初始参数猜测
    initial_guess = [np.max(brightness), np.mean(coordinates[:, 0]), np.mean(coordinates[:, 1]),
                     np.std(coordinates[:, 0]), np.std(coordinates[:, 1])]

    # 拟合高斯曲面
    params, _ = curve_fit(gaussian_surface, coordinates.T, brightness, p0=initial_guess)
    
    return params

# 找到联通的区域并通过高斯曲面拟合求解出它们的位置
def find_bright_regions(image, label, labels):
    
    bright_pixels = np.argwhere(np.abs(labels-label)<1e-5)
    
    # 提取坐标和亮度值
    coordinates = bright_pixels[:, ::-1]
    brightness = image[bright_pixels[:, 0], bright_pixels[:, 1]]
    params = fit_gaussian_surface(coordinates, brightness)
    
    return params

