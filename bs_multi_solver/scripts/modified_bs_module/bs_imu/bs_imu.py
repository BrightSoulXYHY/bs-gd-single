#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


def rpy_to_matrix(rpy):
    '''欧拉角构建旋转矩阵，旋转矩阵为世界系向机体系进行变换'''
    # Z-Y-X的顺序
    # xyz分别对应的角是roll,pitch,yaw

    roll,pitch,yaw = rpy
    R_z = np.array([
        [ np.cos(yaw),  np.sin(yaw), 0],
        [-np.sin(yaw),  np.cos(yaw),  0],
        [ 0,    0,         1]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, -np.sin(pitch)],
        [0, 1, 0],
        [np.sin(pitch), 0,  np.cos(pitch)]
    ])
    R_x = np.array([
        [1,               0,  0],
        [0,  np.cos(roll),  np.sin(roll)],
        [0, -np.sin(roll),  np.cos(roll)],
    ])

    mat = R_x.dot(R_y.dot(R_z))
    return mat

def matrix_to_rpy(R):
    '''旋转矩阵转换为rpy'''
    sy = -R[0,2]
    cycz = R[0,0]
    cysz = R[0,1]
    sxcy = R[1,2]
    cxcy = R[2,2]

    roll = np.arctan2(sxcy, cxcy)
    pitch = np.arcsin(sy)
    yaw = np.arctan2(cysz, cycz)
    return roll, pitch, yaw 

def vec3d_to_ssmatrix(vec):
    '''三维向量转换为反对称矩阵'''
    assert len(vec) == 3, "error: len(vec) != 3"
    x,y,z = vec
    mat = np.array([
        [0,-z,y],
        [z,0,-x],
        [-y,x,0],
    ])
    return mat


def matrix_to_quat(R):
    '''旋转矩阵转四元数'''
    quat = [
        0.5*np.sqrt(1+R[0,0]+R[1,1]+R[2,2]),
        0.5*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])*np.sign(R[2,1]-R[1,2]),
        0.5*np.sqrt(1-R[0,0]+R[1,1]-R[2,2])*np.sign(R[0,2]-R[2,0]),
        0.5*np.sqrt(1-R[0,0]-R[1,1]+R[2,2])*np.sign(R[1,0]-R[0,1]),
    ]
    return quat

def quat_to_matrix(quat):
    '''四元数转旋转矩阵，罗德里格斯公式'''
    w, x, y, z = quat
    v = np.array([x, y, z])
    # R = vvT + (sI+v^)^2
    v_mat = w*np.identity(3)+vec3d_to_ssmatrix(v)
    mat = np.outer(v,v) + v_mat.dot(v_mat)
    return mat

def rot_vec_to_matrix(vec):
    '''旋转向量转为旋转矩阵，罗德里格斯公式'''
    theta = np.linalg.norm(vec)
    n_vec = vec/theta
    matrix = np.cos(theta)*np.identity(3) + (1- np.cos(theta))*np.outer(n_vec, n_vec) + np.sin(theta)*vec3d_to_ssmatrix(n_vec)
    return matrix

def rot_vec_to_quat(vec):
    '''旋转向量转为四元数'''
    theta = np.linalg.norm(vec)
    n_vec = vec/theta
    q0 = np.cos(theta/2)
    qn = n_vec*np.sin(theta/2)
    return [q0] + qn.tolist()

def quat_to_rot_vec(quat):
    '''旋转向量转为四元数'''
    w, x, y, z = quat
    theta = 2*np.arccos(w)
    n_vec = np.array([x, y, z],dtype=float)
    return theta*n_vec

def rpy_to_quat(rpy):
    '''欧拉角转四元数'''
    rpy_np = np.array(rpy)
    cx,cy,cz = np.cos(rpy_np/2)
    sx,sy,sz = np.sin(rpy_np/2)
    quat = [
        cx*cy*cz + sx*sy*sz,
        sx*cy*cz - cx*sy*sz,
        cx*sy*cz + sx*cy*sz,
        cx*cy*sz - sx*sy*cz, 
    ]
    return quat


def quat_to_rpy(quat):
    '''四元数转为欧拉角'''
    w, x, y, z = quat
    sy = -2*(x*z - w*y)
    cycz = 1-2*(y*y+z*z)
    cysz = 2*(w*z+x*y)
    sxcy = 2*(w*x+y*z)
    cxcy = 1-2*(x*x+y*y)
    roll = np.arctan2(sxcy, cxcy)
    pitch = np.arcsin(sy)
    yaw = np.arctan2(cysz, cycz)
    return roll, pitch, yaw




def quat_multi_mat(quat, dir=1):
    '''四元数的乘法矩阵，dir为方向1为左乘，-1为右乘'''
    w, x, y, z = quat
    v = np.array([x, y, z],dtype=float)
    mat = np.zeros((4,4),dtype=float)
    mat[0,0] = w
    mat[0,1:] = -v
    mat[1:,0] = v
    mat[1:,1:] = w*np.identity(3,dtype=float) + dir*vec3d_to_ssmatrix(v)
    return mat
