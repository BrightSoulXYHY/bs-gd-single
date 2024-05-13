#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2023-05-14 15:15:07
# @Author  : BrightSoul (653538096@qq.com)


import os
import time
import numpy
import json


import rospy
import rosbag

from cv_bridge import CvBridge, CvBridgeError

from bs_img_proc import *
from bs_imu_proc import *
from bs_gps_proc import *
from bs_config import *


start_time = time.time()



file_pwd = os.path.dirname(os.path.abspath(__file__))

SAVE_IMG = False

bag_name = "2024-03-30-21-49-36"
input_bag = rosbag.Bag(f"{file_pwd}/../../../bag/{bag_name}.bag")
out_dir = f"{file_pwd}/../../../out/{bag_name}"

if not os.path.exists(out_dir):
    os.makedirs(out_dir)



'''
一样的出真值和计算值
真值记录当前的状态值
ENU位置:/mavros/global_position/global
ENU速度:/mavros/global_position/raw/gps_vel
姿态和角速度:/mavros/imu/data
'''





topicL = [
    "/mavros/global_position/local",
    "/mavros/global_position/global",
    "/mavros/global_position/raw/gps_vel",
    "/mavros/imu/data",
    "/image_ksj_lf",
    "/image_ksj_sf",
]



# 原点
origin_LLA = [
    np.deg2rad( 39.976912),
    np.deg2rad(116.343352),
    37.796408
]

# 过程中会用到的变量
lf_data_cnt = 0
sf_data_cnt = 0


gt_dict = {
    "LLA" : None,
    "vel" : None,
    "acc" : None,
    "quat" : None,
    "ang_vel" : None,
}


# gps_gt = GPSLocalPosioin(*origin_LLA)
img_bridge = CvBridge()
lf_data_list = []
sf_data_list = []


for topic, msg, t in input_bag.read_messages(topics=topicL):
    # 更新真值
    if topic == "/mavros/imu/data":
        acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
        ang_vel = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        gt_dict["acc"] = acc
        gt_dict["quat"] = quat
        gt_dict["ang_vel"] = ang_vel
    elif topic == "/mavros/global_position/global":
        LLA = [
            msg.latitude,
            msg.longitude,
            msg.altitude,
        ]
        gt_dict["LLA"] = LLA
    elif topic == "/mavros/global_position/raw/gps_vel":
        vel = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ]
        gt_dict["vel"] = vel

    elif topic == "/image_ksj_lf":
        # 判断是否初始化完成
        if None in gt_dict.keys():
            continue

        img_gray = img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        data_gt_ptL = img_to_ptL(img_gray)
        plane_px_ptL = [i["center"] for i in data_gt_ptL]
        result_dict = solve_plane_pt(plane_px_ptL,cam_K=camK_lf)

        save_result_dict = result_dict.copy()
        if result_dict['img_valid']:
            save_result_dict["r_vec"] = result_dict["r_vec"].tolist()
            save_result_dict["t_vec"] = result_dict["t_vec"].tolist()
            save_result_dict["plane_px_calcL"] = result_dict["plane_px_calcL"].tolist()
        else:


        # 保存
        save_data = {}
        save_data["timestamp"] = t.to_sec()
        save_data["gt"] = gt_dict.copy()
        save_data["calc"] = save_result_dict
        lf_data_list.append(save_data)
        # with open(f"{out_dir}/{lf_data_cnt:05d}_lf.json","w") as fp:
        #     json.dump(save_data,fp,indent=4)

        if SAVE_IMG:
            cv2.imwrite(f"{out_dir}/{lf_data_cnt:05d}_lf.png",img_gray)


        lf_data_cnt += 1
    elif topic == "/image_ksj_sf":
        # 判断是否初始化完成
        if None in gt_dict.keys():
            continue

        img_gray = img_bridge.imgmsg_to_cv2(msg, msg.encoding)
        data_gt_ptL = img_to_ptL(img_gray)
        plane_px_ptL = [i["center"] for i in data_gt_ptL]
        result_dict = solve_plane_pt(plane_px_ptL,cam_K=camK_sf)

        save_result_dict = result_dict.copy()
        if result_dict['img_valid']:
            save_result_dict["r_vec"] = result_dict["r_vec"].tolist()
            save_result_dict["t_vec"] = result_dict["t_vec"].tolist()
            save_result_dict["plane_px_calcL"] = result_dict["plane_px_calcL"].tolist()
        # 保存
        save_data = {}
        save_data["timestamp"] = t.to_sec()
        save_data["gt"] = gt_dict.copy()
        save_data["calc"] = save_result_dict
        sf_data_list.append(save_data)

        # with open(f"{out_dir}/{sf_data_cnt:05d}_sf.json","w") as fp:
        #     json.dump(save_data,fp,indent=4)
        if SAVE_IMG:
            cv2.imwrite(f"{out_dir}/{sf_data_cnt:05d}_sf.png",img_gray)
        sf_data_cnt += 1
    
save_dict = {}
save_dict["lf_data"] = lf_data_list
save_dict["sf_data"] = sf_data_list
with open(f"{out_dir}/result.json","w") as fp:
    json.dump(save_dict,fp,indent=4)
