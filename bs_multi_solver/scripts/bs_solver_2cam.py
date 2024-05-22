#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2
import numpy as np

np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

import rospy
import message_filters

from geometry_msgs.msg import Transform,Vector3,Quaternion
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from bs_imu import bs_imu
from bs_imu import bs_gps_raw
from bs_img import bs_img_real
from bs_cfg_real_5m import *
from bs_lie_solve import  find_matched_pts

'''
输入图像
输出相对的解算值
'''




class BS_Solver:
    def __init__(self):
        lf_img_topic = "/camera_lf/image_dh"
        sf_img_topic = "/camera_sf/image_dh"


        self.lf_last_plane_pts = np.array([])   #保存上一帧点用于配对
        self.lf_is_first_frame = None                  #判断是否为第一帧   used in lf camera
        self.sf_last_plane_pts = np.array([])
        self.sf_is_first_frame = None


        self.lf_img_sub = rospy.Subscriber(lf_img_topic, Image, self.lf_img_cb)
        self.sf_img_sub = rospy.Subscriber(sf_img_topic, Image, self.sf_img_cb)

        self.lf_plane_pub = rospy.Publisher("/bs_solver_2cam/result/lf_plane", Transform , queue_size=1)
        self.lf_drogue_pub = rospy.Publisher("/bs_solver_2cam/result/lf_drogue", Transform , queue_size=1)
        self.sf_plane_pub = rospy.Publisher("/bs_solver_2cam/result/sf_plane", Transform , queue_size=1)
        self.sf_drogue_pub = rospy.Publisher("/bs_solver_2cam/result/sf_drogue", Transform , queue_size=1)
        self.lf_rpe_pub = rospy.Publisher("/bs_solver_2cam/result/lf_rpe", Float32MultiArray , queue_size=1)
        self.sf_rpe_pub = rospy.Publisher("/bs_solver_2cam/result/sf_rpe", Float32MultiArray, queue_size=1)
        self.final_plane_pub = rospy.Publisher("/bs_solver_2cam/result/final_plane", Transform , queue_size=1)
        self.final_drogue_pub = rospy.Publisher("/bs_solver_2cam/result/final_drogue", Transform , queue_size=1)

        self.lf_plane_sub = message_filters.Subscriber("/bs_solver_2cam/result/lf_plane", Transform)
        self.lf_drogue_sub = message_filters.Subscriber("/bs_solver_2cam/result/lf_drogue", Transform)
        self.sf_plane_sub = message_filters.Subscriber("/bs_solver_2cam/result/sf_plane", Transform)
        self.sf_drogue_sub = message_filters.Subscriber("/bs_solver_2cam/result/sf_drogue", Transform)
        self.lf_rpe_sub = message_filters.Subscriber("/bs_solver_2cam/result/lf_rpe", Float32MultiArray)
        self.sf_rpe_sub = message_filters.Subscriber("/bs_solver_2cam/result/sf_rpe", Float32MultiArray)

        self.ts = message_filters.TimeSynchronizer([self.lf_plane_sub, self.lf_drogue_sub, self.lf_rpe_sub, self.sf_plane_sub, self.sf_drogue_sub,
                                                    self.sf_rpe_sub], queue_size=10)   #tongbu message
        self.ts.registerCallback(self.img_cb)





    def lf_img_cb(self,img_msg):
        img_bridge = CvBridge()
        img_gray = img_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_gray = cv2.cvtColor(img_gray,cv2.COLOR_RGB2GRAY)
        img_gray = cv2.undistort(img_gray,camK_lf, distortion_coeffs_lf)  #去除相机成像畸变
        _, img_bin = cv2.threshold(img_gray, 50, 0xff, cv2.THRESH_BINARY)
        # 如果相机倒置需要旋转一下
        # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)
        
        plane_t_vec = [np.nan]*3
        plane_quat = [np.nan]*4
        drogue_t_vec = [np.nan]*3
        plane_rpe = np.nan
        drogue_rpe = np.nan
        
        gt_pts,_ = bs_img_real.img_to_pts(img_bin)
        if self.lf_is_first_frame and len(self.lf_last_plane_pts) == 0:   #第一帧图像
            plane_result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_lf)
            if plane_result_dict["img_valid"]:
                self.lf_is_first_frame = False 
                cnt_plane_pts = plane_result_dict["plane_px_calcL"]

                plane_t_vec = plane_result_dict["t_vec"]
                plane_rpe = plane_result_dict["rpe"]
                plane_quat = bs_imu.rot_vec_to_quat(plane_result_dict["r_vec"])

            
        else:   #除第一帧以外的图像
            match_pt_idxs = find_matched_pts(self.lf_last_plane_pts,gt_pts,max_err = 300)
            cnt_plane_pts = [ ]
            cnt_real_plane_ptL = [ ]
            for match_pt_idx in match_pt_idxs:
                if match_pt_idx is not None:
                    cnt_plane_pts.append(self.lf_last_plane_pts[match_pt_idx])
                    cnt_real_plane_ptL.append(plane_real_ptL[match_pt_idx])
            
            # 至少有4个点则解算飞机特征点
            if len(cnt_plane_pts) > 3:
                plane_rpe,plane_r_vec,plane_t_vec = cv2.solvePnP(
                    np.array(cnt_real_plane_ptL),
                    np.array(cnt_plane_pts), 
                    camK_lf, distCoeffs=None, 
                    flags=cv2.SOLVEPNP_SQPNP
                )
                plane_quat = bs_imu.rot_vec_to_quat(plane_r_vec)

        # 获取剩下的点用来拟合锥套
        set_plane = set(map(tuple, cnt_plane_pts))
        set_all = set(map(tuple, np.array(gt_pts)))
        set_drogue = set_all - set_plane
        drogue_pts = [np.array(item) for item in set_drogue]  
        drogue_result_dict = bs_img_real.solve_drogue(drogue_pts, camera_matrix=camK_sf, d_drogue=d_drogue)
            
        drogue_t_vec = drogue_result_dict["t_vec"]
        drogue_rpe = drogue_result_dict["rpe"]


        self.lf_last_plane_pts = cnt_plane_pts


        plane_msg = Transform()
        plane_msg.translation.x = plane_t_vec[0]
        plane_msg.translation.y = plane_t_vec[1]
        plane_msg.translation.z = plane_t_vec[2]
        plane_msg.rotation.w = plane_quat[0]
        plane_msg.rotation.x = plane_quat[1]
        plane_msg.rotation.y = plane_quat[2]
        plane_msg.rotation.z = plane_quat[3]
        self.lf_plane_pub.publish(plane_msg)
        
        drogue_msg = Transform()
        drogue_msg.translation.x = drogue_t_vec[0]
        drogue_msg.translation.y = drogue_t_vec[1]
        drogue_msg.translation.z = drogue_t_vec[2]
        self.lf_drogue_pub.publish(drogue_msg)

        rpe_msg = Float32MultiArray()
        rpe_msg.data = []
        rpe_msg.data.append(plane_rpe)
        rpe_msg.data.append(drogue_rpe)
        self.lf_rpe_pub.publish(rpe_msg)


    def sf_img_cb(self,img_msg):
        img_bridge = CvBridge()
        img_gray = img_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_gray = cv2.cvtColor(img_gray,cv2.COLOR_RGB2GRAY)
        img_gray = cv2.undistort(img_gray,camK_sf, distortion_coeffs_sf)  #去除相机成像畸变
        _, img_bin = cv2.threshold(img_gray, 50, 0xff, cv2.THRESH_BINARY)
        # 如果相机倒置需要旋转一下
        # sf_img_gray = cv2.rotate(sf_img_gray, cv2.ROTATE_180)
        
        plane_t_vec = [np.nan]*3
        plane_quat = [np.nan]*4
        drogue_t_vec = [np.nan]*3
        plane_rpe = np.nan
        drogue_rpe = np.nan
                
        gt_pts,_ = bs_img_real.img_to_pts(img_bin)
        if self.sf_is_first_frame and len(self.lf_last_plane_pts) == 0:   #第一帧图像
            plane_result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_sf)
            if plane_result_dict["img_valid"]:
                self.sf_is_first_frame = False 
                cnt_plane_pts = plane_result_dict["plane_px_calcL"]

                plane_t_vec = plane_result_dict["t_vec"]
                plane_rpe = plane_result_dict["rpe"]
                plane_quat = bs_imu.rot_vec_to_quat(plane_result_dict["r_vec"])

            
        else:   #除第一帧以外的图像
            match_pt_idxs = find_matched_pts(self.sf_last_plane_pts,gt_pts,max_err = 300)
            cnt_plane_pts = [ ]
            cnt_real_plane_ptL = [ ]
            for match_pt_idx in match_pt_idxs:
                if match_pt_idx is not None:
                    cnt_plane_pts.append(self.sf_last_plane_pts[match_pt_idx])
                    cnt_real_plane_ptL.append(plane_real_ptL[match_pt_idx])
            
            # 至少有4个点则解算飞机特征点
            if len(cnt_plane_pts) > 3:
                plane_rpe,plane_r_vec,plane_t_vec = cv2.solvePnP(
                    np.array(cnt_real_plane_ptL),
                    np.array(cnt_plane_pts), 
                    camK_sf, distCoeffs=None, 
                    flags=cv2.SOLVEPNP_SQPNP
                )
                plane_quat = bs_imu.rot_vec_to_quat(plane_r_vec)

        # 获取剩下的点用来拟合锥套
        set_plane = set(map(tuple, cnt_plane_pts))
        set_all = set(map(tuple, np.array(gt_pts)))
        set_drogue = set_all - set_plane
        drogue_pts = [np.array(item) for item in set_drogue]  
        drogue_result_dict = bs_img_real.solve_drogue(drogue_pts, camera_matrix=camK_sf, d_drogue=d_drogue)
            
        drogue_t_vec = drogue_result_dict["t_vec"]
        drogue_rpe = drogue_result_dict["rpe"]


        self.sf_last_plane_pts = cnt_plane_pts


        plane_msg = Transform()
        plane_msg.translation.x = plane_t_vec[0]
        plane_msg.translation.y = plane_t_vec[1]
        plane_msg.translation.z = plane_t_vec[2]
        plane_msg.rotation.w = plane_quat[0]
        plane_msg.rotation.x = plane_quat[1]
        plane_msg.rotation.y = plane_quat[2]
        plane_msg.rotation.z = plane_quat[3]
        self.sf_plane_pub.publish(plane_msg)
        
        drogue_msg = Transform()
        drogue_msg.translation.x = drogue_t_vec[0]
        drogue_msg.translation.y = drogue_t_vec[1]
        drogue_msg.translation.z = drogue_t_vec[2]
        self.sf_drogue_pub.publish(drogue_msg)

        rpe_msg = Float32MultiArray()
        rpe_msg.data = []
        rpe_msg.data.append(plane_rpe)
        rpe_msg.data.append(drogue_rpe)
        self.sf_rpe_pub.publish(rpe_msg)

    def img_cb(self,lf_plane_msg,lf_drogue_msg,lf_rpe_msg, sf_plane_msg,sf_drogue_msg,sf_rpe_msg):     #get final results of plane and drogue
        # simply try average tonight
        final_plane_msg = Transform()
        final_drogue_msg = Transform()
        if lf_rpe_msg == None and sf_rpe_msg == None:
            final_plane_msg.translation.x = np.nan
            final_plane_msg.translation.y = np.nan
            final_plane_msg.translation.z = np.nan
            final_drogue_msg.translation.x = np.nan
            final_drogue_msg.translation.y = np.nan
            final_drogue_msg.translation.z = np.nan
        elif lf_rpe_msg != None and sf_rpe_msg == None:
            final_plane_msg = lf_plane_msg
            final_drogue_msg = lf_drogue_msg
        elif lf_rpe_msg == None and sf_rpe_msg != None:
            final_plane_msg = sf_plane_msg
            final_drogue_msg = sf_drogue_msg
        else:
            if lf_rpe_msg < sf_rpe_msg:
                final_plane_msg = lf_plane_msg
                final_drogue_msg = lf_drogue_msg
            else:
                final_plane_msg = sf_plane_msg
                final_drogue_msg = sf_drogue_msg

        self.final_plane_pub.publish(final_plane_msg)
        self.final_drogue_pub.publish(final_drogue_msg)


if __name__ == "__main__":
    rospy.init_node('bs_solver_node', anonymous=True)
    bs_solver = BS_Solver()
    rospy.spin()
