#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
# @Date    : 2023-05-14 15:15:07
# @Author  : BrightSoul (653538096@qq.com)

import numpy as np



# https://en.wikipedia.org/wiki/World_Geodetic_System
# 赤道半径（长半径）
WGS84_a = 6378137.0000
# 极半径（短半径）
WGS84_b = 6356752.3142
# 地球扁率f = (a - b) / a = 1 / 298.257223563
WGS84_f = (WGS84_a - WGS84_b) / WGS84_a
# 偏心率
WGS84_e2 = WGS84_f*(2 - WGS84_f)

"""
ECEF坐标系以地球为圆心
X轴指向本初子午线与地球赤道的一个交点
Z轴指向北极
"""



def WGS84_to_ECEF(lat_rad,lon_rad,alt_m):
    '''WGS84转为ECEF'''
    # 曲率半径
    WGS84_N = WGS84_a/np.sqrt(1-WGS84_e2*np.sin(lat_rad)*np.sin(lat_rad))
    X = (WGS84_N+alt_m)*np.cos(lat_rad)*np.cos(lon_rad)
    Y = (WGS84_N+alt_m)*np.cos(lat_rad)*np.sin(lon_rad)
    Z = (WGS84_N*(1-WGS84_e2)+alt_m)*np.sin(lat_rad)
    return np.array([X,Y,Z])


class GPSLocalPosioin:
    def __init__(self,lat_deg,lon_deg,alt_m):
        lat_init = np.deg2rad(lat_deg)
        lon_init = np.deg2rad(lon_deg)
        sin_lat = np.sin(lat_init)
        cos_lat = np.cos(lat_init)
        sin_lon = np.sin(lon_init)
        cos_lon = np.cos(lon_init)
        self.mat_S = np.array([
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
        ])



        self.ecef_init = WGS84_to_ECEF(lat_init,lon_init,alt_m)
    
    def update_pose(self,lat_deg,lon_deg,alt_m):
        ecef_cnt = WGS84_to_ECEF(np.deg2rad(lat_deg),np.deg2rad(lon_deg),alt_m)
        enu_xyz = self.mat_S.dot(ecef_cnt - self.ecef_init)
        return enu_xyz
  

if __name__ == "__main__":
    origin_LLA = [
        39.976912,
        116.343352,
        37.796408
    ]

    lat = 39.976493
    lon = 116.343370
    alt = 38.116759

    taper_pose = GPSLocalPosioin(*origin_LLA)
    enu_pose = taper_pose.update_pose(lat,lon,alt)
    print(enu_pose)

