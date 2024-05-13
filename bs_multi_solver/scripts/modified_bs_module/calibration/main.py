#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 多相机联合标定、一维标定物
import numpy as np
import cv2
import sys,os
# 当前文件父级的父级的绝对路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bs_img.bs_img_real import *
from solve_fundemental_matrix import *
from solve_Intrinsics_matrix import *
from solve_external_parameters import *
from solve_reprojection_error import *
from solve_angle import *
from bundle_adjustment import *

def calib(path,N,AC,BC,img1_width,img1_height,img2_width,img2_height):

    pts_pixel1 = np.array([[]])   #1号相机
    pts_pixel2 = np.array([])   #2号相机
    for i in range(0,N):
        img1 = cv2.imread(path+"1 ("+str(i)+").jpg")
        img_gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
        gt_pts1,_ = img_to_pts(img_gray1,min_area=5,max_area=444444444,threshold=200)
        pts_pixel1=np.append(pts_pixel1,gt_pts1)
        img2 = cv2.imread(path+"2 ("+str(i)+").jpg")
        img_gray2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        gt_pts2,_ = img_to_pts(img_gray2,min_area=5,max_area=444444444,threshold=200)
        pts_pixel2=np.append(pts_pixel2,gt_pts2)
    pts_pixel1 = np.reshape(np.array(pts_pixel1, dtype=np.float32),(-1,2))
    pts_pixel2 = np.reshape(np.array(pts_pixel2, dtype=np.float32),(-1,2))  
    F = getFundamentalMatrix(pts_pixel1,pts_pixel2)

    
    K1,K2 = getIntrinsics(img1_width,img1_height,img2_width,img2_height,F)  #求解出两个相机的内参矩阵
    
    R,t = getRt(K1,K2,F,pts_pixel1,pts_pixel2,AC)
    
    P1_tmp=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]], dtype=np.float64)
    P1=np.dot(K1,P1_tmp)    #相机1投影矩阵
    P2_tmp=np.array([[R[0,0],R[0,1],R[0,2],t[0,0]],[R[1,0],R[1,1],R[1,2],t[1,0]],[R[2,0],R[2,1],R[2,2],t[2,0]]], dtype=np.float64)  
    P2=np.dot(K2,P2_tmp)    #相机2投影矩阵
    # 三维重建初始化各个点三维坐标
    # points1, points2：为1号与2号两台相机拍摄的标定杆上标定球的中心的像素点坐标。
    # 数据排列方式与求解基础矩阵的数据排列一样（就是一个东西）
    Point3D = get3Dpoint(pts_pixel1,pts_pixel2,P1,P2)
    
    average_reproject_error1,average_reproject_error2,average_distance_AB,average_distance_BC = get_average_error(pts_pixel1,pts_pixel2,P1,P2,Point3D,flag=0)
    angle,Point3DA = getAngle(AC,BC,Point3D)
    
    #Bundle Adjustment
    K_all = np.array([])  # 将相机内参存入
    K_all = np.append(K_all,K1)
    K_all = np.append(K_all,K2)
    K_all = np.reshape(K_all,(2,3,3))
    R0 = np.eye(3)
    t0 = np.zeros((3,1))
    R_all = np.array([])
    R_all = np.append(R_all,R0)
    R_all = np.append(R_all,R)    #getRt求解结果
    R_all = np.reshape(R_all,(2,3,3))

    t_all = np.array([])
    t_all = np.append(t_all,t0)
    t_all = np.append(t_all,t)    #同样为getRt求解结果
    t_all = np.reshape(t_all,(2,3))

    pts_pixel = np.array([])
    pts_pixel = np.append(pts_pixel,pts_pixel1)
    pts_pixel = np.append(pts_pixel,pts_pixel2)
    pts_pixel = np.reshape(pts_pixel,(2,-1,2))

    
    intrinsic_optimized, extrinsics_optimized = bundle_adjustment(K=K_all,R=R_all,t=t_all,true_point3D_A=Point3DA,angle=angle,
                                                                  points=pts_pixel,AC=AC,BC=BC)
    
    
    
    return   

if __name__ == "__main__":
    path="C:\\Users\\12590\\Desktop\\test\\"
    N=6        #用于标定图片总数
    AC = 0.2    # 假设标定杆A点和C点之间的距离
    BC = 0.1
    img1_width = 2448
    img2_width = 2448
    img1_height = 2048
    img2_height = 2048
    calib(path,N,AC,BC,img1_width,img1_height,img2_width,img2_height)
