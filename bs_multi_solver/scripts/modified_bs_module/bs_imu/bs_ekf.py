#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from modified_bs_module.bs_imu.bs_imu import quat_multi_mat



class RelativeEKF:
    R_default = np.diag([0.5,0.5,0.5,0.2,0.2,0.2,0.5,0.5,0.5,0.5])
    Q_default = 50*np.diag([0.5,0.5,0.5,0.5,0.5,0.5,0.5])
    P_default = np.diag([0.5,0.5,0.5,0.2,0.2,0.2,0.05,0.05,0.05,0.05])
    
    # 发散或者未初始化处于wait状态
    # 运行状态中有nan则重新初始化
    EKF_WAIT = 0
    EKF_RUN = 1
    
    def __init__(self):
        # 状态变量pos,vel,quat
        
        # 初始化维度
        self.dim = 10

        # 状态噪声和观测噪声
        self.R = self.R_default
        self.Q = self.Q_default
        


        # 状态变量
        self.X_pre = None
        self.P_pre = self.P_default
        
        self.X_post = None
        self.P_post = None
        
        self.EKF_state = self.EKF_WAIT


    def set_noise_matrix(self,R,Q):
        # 设置噪声矩阵
        self.R = R
        self.Q = Q

    def init_state(self,X):
        # 初始化状态变量
        self.X_pre = X
        self.P_pre = self.P_default

    def clac_matirx_F(self,dt,angle_vel):
        # 对应于状态变量pos,vel,quat的矩阵F
        # 构造F
        assis_quat = np.array([0, angle_vel[0], angle_vel[1], angle_vel[2]])
        F = np.identity(self.dim)
        F[:3,3:6] = np.identity(3)*dt
        F[6:,6:] = np.identity(4) + 0.5*dt*quat_multi_mat(assis_quat,-1)
        return F

    def clac_matirx_H(self):
        # 构造H
        H = np.zeros((7,10))
        H[:3,:3] = np.identity(3)
        H[-4:,-4:] = np.identity(4)
        return H
    
    def predict(self,dt,lin_acc,angle_vel):
        F = self.clac_matirx_F(dt,angle_vel)
        uk = np.zeros(10)
        uk[3:6] = -lin_acc*dt

        # 预测
        self.X_post = F.dot(self.X_pre) + uk
        self.P_post = F.dot(self.P_pre.dot(F.T))+self.R

    def update(self,Z):
        # 更新
        H = self.clac_matirx_H()

        inv_K = np.linalg.inv(H.dot(self.P_post.dot(H.T))+self.Q)
        K = np.dot(self.P_post.dot(H.T),inv_K)
 
        self.X_pre = self.X_post + np.dot(K,Z-H.dot(self.X_post))
        self.P_pre = np.dot(np.identity(self.dim) - K.dot(H),self.P_post)
    
    def is_run(self):
        '''
            未初始化或者发散返回False
            正常运行返回True
        '''
        return self.EKF_state
    
    
    def check_run(self):
        '''
            检查滤波器是否发散
        '''
        return (
            np.isnan(self.X_post).any() or
            np.isnan(self.X_pre).any() or
            np.isnan(self.P_post).any() or
            np.isnan(self.P_pre).any()
        )
    
