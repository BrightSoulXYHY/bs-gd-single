#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

import rospy
import message_filters

from geometry_msgs.msg import Transform,PoseWithCovarianceStamped
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix



from bs_imu import bs_imu
from bs_imu import bs_gps_raw
from bs_img import bs_img_real
from bs_img.bs_cfg_real_1m import *

from collections import deque

'''
接收GPS信号
接受IMU信号
接受视觉解算结果
对比真值和实际值
'''




class BS_ResultViewer:
    def __init__(self):
        # 每1s打印一次
        rospy.Timer(rospy.Duration(1), self.timer_cb)


        # GPS信息作为真值
        self.init_cnt = 0
        self.gps_state = False
        self.gps_data_queue = deque(maxlen=10)

        self.gps_calc = None
        self.gps_pose_enu = None
        # 当GPS定位到一定范围内开始解算
        self.gps_data_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.gps_data_cb)


        # 从靶标坐标系到相机系的转换
        self.t_ct_lf = [np.nan]*3
        self.R_ct_lf = [np.nan]*3
        self.t_ct_sf = [np.nan]*3
        self.R_ct_sf = [np.nan]*3
        # 解算值
        self.lf_res_sub = rospy.Subscriber("bs_debuger/result/lf", Transform, self.lf_res_cb)
        self.sf_res_sub = rospy.Subscriber("bs_debuger/result/sf", Transform, self.sf_res_cb)

        # 无人机解算的位置
        self.R_be = None
        self.mav_pose_enu_init = np.zeros(3)
        self.mav_pose_enu = None
        self.local_pose_sub = rospy.Subscriber("mavros/global_position/local", Odometry, self.local_pose_sub)


    def gps_data_cb(self,msg):
        gps_data = [
            msg.latitude,
            msg.longitude,
            msg.altitude,
        ]
        self.gps_data_queue.append(gps_data)
        if self.gps_state:
            self.gps_pose_enu = self.gps_calc.update_pose(*gps_data)
    
    def lf_res_cb(self,msg):
        self.t_ct_lf = np.array([
            trans_msg.translation.x,
            trans_msg.translation.y,
            trans_msg.translation.z,
        ])
        self.R_ct_lf = bs_imu.quat_to_matrix([
            trans_msg.rotation.w,
            trans_msg.rotation.x,
            trans_msg.rotation.y,
            trans_msg.rotation.z,
        ])
    

    def sf_res_cb(self,msg):
        self.t_ct_sf = np.array([
            trans_msg.translation.x,
            trans_msg.translation.y,
            trans_msg.translation.z,
        ])
        self.R_ct_sf = bs_imu.quat_to_matrix([
            trans_msg.rotation.w,
            trans_msg.rotation.x,
            trans_msg.rotation.y,
            trans_msg.rotation.z,
        ])


    def local_pose_sub(self,msg):
        self.R_be = bs_imu.quat_to_matrix([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])
        self.mav_pose_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]) - self.mav_pose_enu_init


    def timer_cb(self,event):
        if self.gps_state:
            tgt_pose_lf = np.dot(self.R_be.T,R_bc.dot(self.t_ct_lf))
            tgt_pose_sf = np.dot(self.R_be.T,R_bc.dot(self.t_ct_sf))
            print(f"mav_pose_enu:{self.mav_pose_enu} gps_pose_enu:{self.gps_pose_enu}")
            print(f"tgt_pose_lf:{tgt_pose_lf} tgt_pose_sf:{tgt_pose_sf}")
        else:
            if len(self.gps_data_queue) > 5:
                gps_init = self.gps_data_queue[0]
                gps_calc = bs_gps_raw.GPSLocalPosioin(*gps_init)
                pose_enu_list = np.array([
                    gps_calc.update_pose(*i)
                    for i in self.gps_data_queue
                ])
                pose_enu_err = np.average(pose_enu_list[1:],axis=0)
                print(pose_enu_err)

                if np.max(pose_enu_err[:2]) < 0.05:
                    self.init_cnt += 1
                else:
                    self.init_cnt = 0


                if self.init_cnt > 15:
                    self.gps_state = True
                    self.gps_calc = bs_gps_raw.GPSLocalPosioin(*gps_init)
                    self.mav_pose_enu_init = self.mav_pose_enu
                    print("init done!!")
            




if __name__ == "__main__":
    rospy.init_node('bs_result_viewer_node', anonymous=True)
    bs_result_viewer = BS_ResultViewer()
    rospy.spin()
