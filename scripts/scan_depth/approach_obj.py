#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#テーブルに近づくROSノード
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
scan_depth = LaserScan()
flag_start = 0


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_depth(sub_scan_depth):
    global scan_depth, flag_start
    scan_depth = sub_scan_depth
    flag_start = 1


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
        #commonf_speech_multi('オブジェクトに近づきます。')

        commonf_actionf_cam_lift(0.555)
        commonf_pubf_cam_tilt(1.2)
        commonf_pubf_cam_pan(0)

        rospy.Subscriber("/scan/depth", LaserScan, subf_scan_depth)

        main_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if flag_start == 1:
                rospy.loginfo(scan_depth.ranges[int(len(scan_depth.ranges) / 2)])
                if scan_depth.ranges[int(len(scan_depth.ranges) / 2 - 20)] > 0.7 and \
                   scan_depth.ranges[int(len(scan_depth.ranges) / 2)] > 0.7 and \
                   scan_depth.ranges[int(len(scan_depth.ranges) / 2 + 20)] > 0.7:
                    commonf_pubf_cmd_vel(0.15, 0, 0, 0, 0, 0)
                else:
                    commonf_pubf_cmd_vel(0.1, 0, 0, 0, 0, 0)
                    rospy.sleep(1.2)
                    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

                    #commonf_speech_multi('オブジェクトに近づきました。')

                    sys.exit(0)        

            main_rate.sleep()