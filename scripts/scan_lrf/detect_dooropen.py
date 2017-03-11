#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#ドアオープンを検出するROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/12
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


#--------------------------------------------------
#グローバル変数
#--------------------------------------------------
scan_lrf = LaserScan()
flag_start = 0


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_lrf(sub_scan_lrf):
    global scan_lrf, flag_start
    scan_lrf = sub_scan_lrf
    flag_start = 1


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
        commonf_speech_multi('ドアオープンを検出します。')

        rospy.Subscriber("/scan/lrf", LaserScan, subf_scan_lrf)

        main_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if flag_start == 1:
                rospy.loginfo(scan_lrf.ranges[int(len(scan_lrf.ranges) / 2)])
                if scan_lrf.ranges[int(len(scan_lrf.ranges) / 2)] < 2.0:
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                else:
                    commonf_speech_single('ドアオープンを検出しました。２メートル前進します。')
                    rospy.sleep(2)

                    commonf_pubf_cmd_vel(0.4, 0, 0, 0, 0, 0)
                    rospy.sleep(5)
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                    
                    sys.exit(0)        

            main_rate.sleep()
