#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2023-05-14 15:15:07
# @Author  : BrightSoul (653538096@qq.com)


import os
import time
import numpy as np
import json
import cv2

import rospy
import rosbag

from cv_bridge import CvBridge, CvBridgeError

from bs_imu import bs_imu
from bs_img import bs_img_real
from bs_cfg_real_5m import *

np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

start_time = time.time()



file_pwd = os.path.dirname(os.path.abspath(__file__))
out_dir = f"{file_pwd}/../../../out"

if not os.path.exists(out_dir):
    os.makedirs(out_dir)



def epoch_once(bag_name,out_dir):
    input_bag = rosbag.Bag(f"{file_pwd}/../../../bag/{bag_name}.bag")

    data_cnt = 0
    img_bridge = CvBridge()
    lf_data_list = []
    sf_data_list = []


    for topic, msg, t in input_bag.read_messages(topics=["/image_dh_sf"]):
        # 更新真值
        if topic == "/image_dh_sf":
            img_gray = img_bridge.imgmsg_to_cv2(msg, msg.encoding)
            img_gray = cv2.cvtColor(img_gray,cv2.COLOR_RGB2GRAY)
            _, img_bin = cv2.threshold(img_gray, 50, 0xff, cv2.THRESH_BINARY)
            # 如果相机倒置需要旋转一下
            # lf_img_gray = cv2.rotate(lf_img_gray, cv2.ROTATE_180)
            gt_pts,_ = bs_img_real.img_to_pts(img_bin)
            result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_sf)
            t_vec = result_dict["t_vec"]
            sf_data_list.append(t_vec)
            data_cnt += 1
        
            # print(f"{data_cnt} done: {t_vec}")
    np.savetxt(f"{out_dir}/{bag_name}.txt",sf_data_list,fmt="%.6f")
    return np.array(sf_data_list)
    # print(f"all done")


if __name__ == "__main__":
    for i in range(20,230,10):
        bag_name = f"{i}m"
        t_vec_data = epoch_once(bag_name,out_dir)
        t_vec = np.nanmean(t_vec_data,axis=0)
        gt_dis = i/2*2.05
        print(f"[{i}m] real:{gt_dis:.2f} calc:{t_vec}")
