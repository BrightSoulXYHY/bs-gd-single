#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

import rospy
import message_filters

from geometry_msgs.msg import Transform,Vector3,Quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from bs_imu import bs_imu
from bs_imu import bs_gps_raw
from bs_img import bs_img_real
from bs_img.bs_cfg_real_5m import *
from bs_lie_solve import *

'''
输入图像
输出相对的解算值
'''




class BS_Solver:
    def __init__(self):
        lf_img_topic = "image_dh_lf"
        sf_img_topic = "image_dh_sf"
        self.last_plane_pts = np.array([])   #保存上一帧点用于配对
        self.is_first_frame = None                  #判断是否为第一帧

        self.img_bridge = CvBridge()

        # self.lf_img_sub = rospy.Subscriber(lf_img_topic, Image, self.lf_img_cb)
        self.sf_img_sub = rospy.Subscriber(sf_img_topic, Image, self.sf_img_cb)

        # self.lf_res_pub = rospy.Publisher("/bs_debuger/result/lf", Transform , queue_size=1)
        self.sf_plane_pub = rospy.Publisher("/bs_debuger/result/sf_plane", Transform , queue_size=1)
        self.sf_drogue_pub = rospy.Publisher("/bs_debuger/result/sf_drogue", Transform , queue_size=1)

        # self.img_res_pub = rospy.Publisher("/bs_debuger/image_res", Image, queue_size=1)


        # self.ts = message_filters.TimeSynchronizer([self.lf_img_sub, self.sf_img_sub], queue_size=10)
        # self.ts.registerCallback(self.img_cb)
    
    # def lf_img_cb(self,img_msg):
    #     img_gray = self.img_bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
    #     # 如果相机倒置需要旋转一下
    #     # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)

    #     gt_pts,_ = bs_img_real.img_to_pts(img_gray)
    #     result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_lf)
        
    #     x,y,z = result_dict["t_vec"]
    #     qw,qx,qy,qz = bs_imu.rot_vec_to_quat(result_dict["r_vec"])

    #     trans_msg = Transform()
    #     trans_msg.translation.x = x
    #     trans_msg.translation.y = y
    #     trans_msg.translation.z = z
    #     trans_msg.rotation.w = qw
    #     trans_msg.rotation.x = qx
    #     trans_msg.rotation.y = qy
    #     trans_msg.rotation.z = qz
    #     self.lf_res_pub.publish(trans_msg)



    def sf_img_cb(self,img_msg):
        img_gray = self.img_bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
        img_gray = cv2.cvtColor(img_gray,cv2.COLOR_RGB2GRAY)
        _, img_bin = cv2.threshold(img_gray, 50, 0xff, cv2.THRESH_BINARY)
        # 如果相机倒置需要旋转一下
        # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)
        
        plane_t_vec = [np.nan]*3
        plane_quat = [np.nan]*4
        drogue_t_vec = [np.nan]*3
        
        gt_pts,_ = bs_img_real.img_to_pts(img_bin)
        if self.is_first_frame:   #第一帧图像
            plane_result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_sf)
            if plane_result_dict["img_valid"]:
                self.is_first_frame = False 
                self.last_plane_pts = plane_result_dict["plane_px_calcL"]

                plane_t_vec = plane_result_dict["t_vec"]
                plane_quat = bs_imu.rot_vec_to_quat(plane_result_dict["r_vec"])

            
        else:   #除第一帧以外的图像
            match_pt_idxs = find_matched_pts(self.last_plane_pts,gt_pts,max_err = 300)
            cnt_plane_pts = [ ]
            cnt_real_plane_ptL = [ ]
            for match_pt_idx in match_pt_idxs:
                if match_pt_idx is not None:
                    cnt_plane_pts.append(self.last_plane_pts[match_pt_idx])
                    cnt_real_plane_ptL.append(plane_real_ptL[match_pt_idx])
            
            # 至少有4个点则解算飞机特征点
            if len(cnt_plane_pts) > 3:
                
                retval,plane_r_vec,plane_t_vec = cv2.solvePnP(
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
            
            self.last_plane_pts = cnt_plane_pts


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





if __name__ == "__main__":
    rospy.init_node('bs_solver_node', anonymous=True)
    bs_solver = BS_Solver()
    rospy.spin()
