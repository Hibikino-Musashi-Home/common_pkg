#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#scan_lrfとscan_depthの合成を制御するROSノード
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
scan_depth = LaserScan()
scan_mode = String()


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_lrf(sub_scan_lrf):
    global scan_lrf
    scan_lrf = sub_scan_lrf


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_depth(sub_scan_depth):
    global scan_depth
    scan_depth = sub_scan_depth


#--------------------------------------------------
#--------------------------------------------------
def subf_scan_mode(sub_scan_mode):
    global scan_mode
    scan_mode = sub_scan_mode
    rospy.loginfo('[marge_scan]: Mode was changed: %s', sub_scan_mode.data)


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    rospy.Subscriber("/scan/lrf", LaserScan, subf_scan_lrf)
    rospy.Subscriber("/scan/depth", LaserScan, subf_scan_depth)
    rospy.Subscriber("/scan/mode", String, subf_scan_mode)
    pub_scan = rospy.Publisher('/scan', LaserScan, queue_size = 1)

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if scan_mode.data == 'lrf':
            pub_scan.publish(scan_lrf)
        elif scan_mode.data == 'depth':
            pub_scan.publish(scan_depth)
        elif scan_mode.data == 'marge':
            scan_marge = LaserScan()
            scan_marge = scan_lrf
            scan_marge_ranges_list = list(scan_marge.ranges)

            for i in range(0, int((abs(scan_depth.angle_min) + abs(scan_depth.angle_max)) / scan_depth.angle_increment)):
                min_diff = abs(scan_lrf.angle_min) + abs(scan_lrf.angle_max)
                cur_depth_angle = scan_depth.angle_min + scan_depth.angle_increment * i
                for j in range(int(abs(scan_lrf.angle_min - cur_depth_angle) / scan_lrf.angle_increment) - 20, int(abs(scan_lrf.angle_min - cur_depth_angle) / scan_lrf.angle_increment) + 20):
                    cur_lrf_angle = scan_lrf.angle_min + scan_lrf.angle_increment * j
                    if min_diff > abs(cur_depth_angle - cur_lrf_angle):
                        min_diff = abs(cur_depth_angle - cur_lrf_angle)
                        min_diff_lrf_angle_num = j
                if scan_depth.ranges[i] < 4:
                    if scan_depth.ranges[i] < scan_lrf.ranges[min_diff_lrf_angle_num]:
                        scan_marge_ranges_list[min_diff_lrf_angle_num] = scan_depth.ranges[i]
                
            scan_marge.ranges = tuple(scan_marge_ranges_list)
            pub_scan.publish(scan_marge)
        else:
            pub_scan.publish(scan_lrf)

        main_rate.sleep()
