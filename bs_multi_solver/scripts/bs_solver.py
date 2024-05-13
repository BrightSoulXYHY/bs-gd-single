#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

import rospy
import message_filters

from geometry_msgs.msg import Transform,Vector3,Quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from bs_imu import bs_imu
from bs_imu import bs_gps_raw
from bs_img import bs_img_real
from bs_img.bs_cfg_real_1m import *

'''
输入图像
输出相对的解算值
'''




class BS_Solver:
    def __init__(self):
        lf_img_topic = "image_ksj_lf"
        sf_img_topic = "image_ksj_sf"


        self.img_bridge = CvBridge()

        self.lf_img_sub = rospy.Subscriber(lf_img_topic, Image, self.lf_img_cb)
        self.sf_img_sub = rospy.Subscriber(sf_img_topic, Image, self.sf_img_cb)

        self.lf_res_pub = rospy.Publisher("/bs_debuger/result/lf", Transform , queue_size=1)
        self.sf_res_pub = rospy.Publisher("/bs_debuger/result/sf", Transform , queue_size=1)

        # self.img_res_pub = rospy.Publisher("/bs_debuger/image_res", Image, queue_size=1)


        # self.ts = message_filters.TimeSynchronizer([self.lf_img_sub, self.sf_img_sub], queue_size=10)
        # self.ts.registerCallback(self.img_cb)
    
    def lf_img_cb(self,img_msg):
        img_gray = self.img_bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
        # 如果相机倒置需要旋转一下
        # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)

        gt_pts,_ = bs_img_real.img_to_pts(img_gray)
        result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_lf)
        
        x,y,z = result_dict["t_vec"]
        qw,qx,qy,qz = bs_imu.rot_vec_to_quat(result_dict["r_vec"])

        trans_msg = Transform()
        trans_msg.translation.x = x
        trans_msg.translation.y = y
        trans_msg.translation.z = z
        trans_msg.rotation.w = qw
        trans_msg.rotation.x = qx
        trans_msg.rotation.y = qy
        trans_msg.rotation.z = qz
        self.lf_res_pub.publish(trans_msg)



    def sf_img_cb(self,img_msg):
        img_gray = self.img_bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
        # 如果相机倒置需要旋转一下
        # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)

        gt_pts,_ = bs_img_real.img_to_pts(img_gray)
        result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_sf)
        
        x,y,z = result_dict["t_vec"]
        qw,qx,qy,qz = bs_imu.rot_vec_to_quat(result_dict["r_vec"])

        trans_msg = Transform()
        trans_msg.translation.x = x
        trans_msg.translation.y = y
        trans_msg.translation.z = z
        trans_msg.rotation.w = qw
        trans_msg.rotation.x = qx
        trans_msg.rotation.y = qy
        trans_msg.rotation.z = qz
        self.sf_res_pub.publish(trans_msg)





if __name__ == "__main__":
    rospy.init_node('bs_solver_node', anonymous=True)
    bs_solver = BS_Solver()
    rospy.spin()
