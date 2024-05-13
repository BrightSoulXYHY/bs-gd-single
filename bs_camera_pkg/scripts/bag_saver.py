#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from mavros_msgs.msg import RCIn


import os
import time

from collections import OrderedDict

import subprocess
import signal

'''
CH7设置为高开始记录bag
CH7设置为低停止记录bag
'''


file_pwd = os.path.dirname(os.path.abspath(__file__))
bag_path = "{}/../../../bag".format(file_pwd)
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")

if not os.path.exists(bag_path):
    os.mkdir(bag_path)



def terminate_ros_node(name):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read().decode('utf-8')
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for line in list_output.split("\n"):
        if line.startswith(name):
            os.system(f"rosnode kill {line}")


class BagSaver:
    def __init__(self):
        self.arm_state = False
        self.rcin_sub = rospy.Subscriber("/mavros/rc/in", RCIn, self.rcin_callback)


        self.rc_dict = OrderedDict()
        for i in range(5,10):
            self.rc_dict["ch{}".format(i)] = -1

    def rcin_callback(self, msg):
        def pwm_map(pwm):
            # 0-1300->0/1301-1700->1/1701-2000->2
            result = None
            if 0 < pwm <= 1300:
                result = 0
            if 1300 < pwm <= 1700:
                result = 1
            if 1700 < pwm <= 2100:
                result = 2
            return result

        last_rc_dict = self.rc_dict.copy()
        rc_chL = msg.channels
        
        for k,v in zip(last_rc_dict.keys(),rc_chL[4:10]):
            self.rc_dict[k] = pwm_map(v)

        if self.rc_dict != last_rc_dict:
            print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}".format(*self.rc_dict.values()))
    
    def run(self):
        # 等待CH7为高
        while self.rc_dict["ch7"] != 2:
            time.sleep(0.1)
        
        # 开始记录数据
        save_proc = subprocess.Popen(rosbag_cmd,shell=True,cwd=bag_path)
        print("start record")

        # pid = save_proc.pid
        
        
        # CH7为低则停止记录
        while self.rc_dict["ch7"] == 2:
            time.sleep(1)
        print("stop record")
        terminate_ros_node("/record")




if __name__=="__main__":
    rospy.init_node("bag_save_node")


    rosbag_cmd = "rosbag record -a"
    
    print(os.path.abspath(bag_path))
    bag_saver = BagSaver()
    bag_saver.run()



    
 


    