#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
input1、input2:为1号与2号两台相机拍摄的标定杆上标定球的中心的像素点坐标。数据的排列方式:
假如1号相机拍摄了80张标定杆图片,那么input1里面的数据为A1B1C1A2B2C2A3B3C3...A80B80C80。
其中A1B1C1表示为这是第一张图片的A小球的中心坐标像素点、B小球的中心坐标像素点、C小球的中心坐标像素点
其中A2B2C2表示为这是第二张图片的A小球的中心坐标像素点、B小球的中心坐标像素点、C小球的中心坐标像素点,后面的同理
2号相机的数据排列与1号相机一样,两台相机的数据要保证对应上。
F:返回的基础矩阵
'''
import cv2
import numpy as np
from typing import List
from numpy.linalg import inv

def getFundamentalMatrix(input1: List[np.ndarray], input2: List[np.ndarray]) -> np.ndarray:
    
    cvPoints1 = np.reshape(np.array(input1, dtype=np.float32),(-1,2))
    cvPoints2 = np.reshape(np.array(input2, dtype=np.float32),(-1,2))

    F, _ = cv2.findFundamentalMat(np.array(cvPoints1), np.array(cvPoints2), cv2.FM_8POINT)


    return F

