#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import itertools


from modified_bs_module.bs_img import bs_img_real
from modified_bs_module.bs_img.bs_cfg_real_1m import *


import argparse


def parse_args():
    parser = argparse.ArgumentParser(description='test bs_img')
    parser.add_argument('img_path', help='image file path')
    args = parser.parse_args()
    return args

def main():
    args = parse_args()

    img_path = args.img_path
    img_gray = cv2.imread(img_path,cv2.IMREAD_UNCHANGED)
    gt_pts,_ = bs_img_real.img_to_pts(img_gray,min_area=15, threshold=220)
    result_dict = bs_img_real.solve_plane_pt(gt_pts,plane_real_ptL=plane_real_ptL,cam_K=camK_lf)
    t_vec = result_dict["t_vec"]
    r_vec = result_dict["r_vec"]
    pose = bs_img_real.vec_to_enu(t_vec,r_vec,R_wt)
    print(t_vec)
    print(r_vec)
    print(pose)


if __name__ == '__main__':
    main()

