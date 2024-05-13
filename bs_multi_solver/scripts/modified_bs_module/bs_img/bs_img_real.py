#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np

from bs_img.bs_cfg_real_1m import *
from bs_img.bs_img_base import *
from bs_img.gauss_fit import *
from bs_img.ellipse_fit import *


def img_to_pts(img_gray,min_area, max_area ,threshold):
    '''提取图片中的特征点'''
    # img_blurred = cv2.GaussianBlur(img_gray, (5, 5), 0)
    _, img_bin = cv2.threshold(img_gray, threshold, 0xff, cv2.THRESH_BINARY)
    # cv2.namedWindow('1',0)
    # cv2.resizeWindow('1',(1920,1080))
    # cv2.imshow('1',img_bin)
    # cv2.waitKey(0)
    
    gt_pts = []
    area_list = []
    #版本1-3：：：
    #返回联通区域总数、序号、像素个数、质心
    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_bin, connectivity=4)
    #序号为0是背景
    for stat,centroid,label in zip(stats[1:],centroids[1:],range(1,np.max(labels)+1)):
        if stat[-1] > max_area or stat[-1] < min_area:
            continue
        
        '''
        时间  版本2最快
        精度  版本2、3相当，优于版本1
        '''
        
        # # 版本1：使用高斯曲面拟合灰度值的方法求LED位置   过曝可能就不行了吧 时间比其他两个版本多多了
        # params = find_bright_regions(img_gray, label, labels)
        # area_list.append(stat[-1])
        # gt_pts.append(np.array([params[1],params[2]]))
        
        # 版本2：求区域质心坐标  连通域分析以求  
        gt_pts.append(centroid) 
        
        # # # 版本3：椭圆拟合边界
        # mask = np.uint8(labels == label)
        # center = ellipse_fit(mask)
        # gt_pts.append(np.array([center[0][0],center[0][1]]))
        
    # # 版本4： Canny+椭圆拟合改进版    感觉不怎么稳定
    # edges = cv2.Canny(img_gray, 30, 100, apertureSize=3, L2gradient=True)
    # # cv2.namedWindow('1',0)
    # # cv2.resizeWindow('1',(1920,1080))
    # # cv2.imshow('1',edges)
    # # cv2.waitKey(0)
    # contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # for contour in contours:
    #     if len(contour) < 5:
    #         continue
    #     ellipse = cv2.fitEllipse(contour)
    #     center = ellipse[0]
    #     gt_pts.append(np.array(center))
        

    return gt_pts,area_list
    

    
