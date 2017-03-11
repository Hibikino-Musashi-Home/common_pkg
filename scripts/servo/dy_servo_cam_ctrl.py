#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#カメラパンチルトROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/08
#
#memo:
#1[rad] = 57.325[deg]
#0.017[rad] = 1[deg]
#
#pan
#+3.14[rad](サーボ出力軸反時計回り) : 0[rad](サーボ正面) : -3.14[rad](サーボ出力軸時計回り)
#tilt
#+3.14[rad](サーボ出力軸反時計回り) : 0[rad](サーボ正面) : -3.14[rad](サーボ出力軸時計回り)
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


#--------------------------------------------------
#定数
#--------------------------------------------------
RAD2DEG = 180 / 3.14 #[rad]から[deg]に変換
DEG2RAD = 3.14 / 180 #[deg]から[rad]に変換
INIT_PAN = 0.0 #pan初期値[rad]
OFFSET_PAN = 0.035 #pan初期ズレ補正値[rad] 2[deg]
MAX_LIM_PAN = 0.872 #pan最大限界値[rad] 50[deg]
MIN_LIM_PAN = -0.872 #pan最小限界値[rad] -50[deg]
INIT_TILT = 0.0 #tilt初期値[rad]
OFFSET_TILT = 0.035 #tilt初期ズレ補正値[rad] 2[deg]
MAX_LIM_TILT = 1.57 #tilt最大限界値[rad] 90[deg]
MIN_LIM_TILT = -0.174 #tilt最小限界値[rad] -10[deg]


#--------------------------------------------------
#グローバル変数
#--------------------------------------------------
i_pan = INIT_PAN #他ノードからsubscribeするpan値
i_tilt = INIT_TILT #他ノードからsubscribeするtilt値


#--------------------------------------------------
#pan入力関数
#--------------------------------------------------
def subf_pan(sub_pan_data):
    global i_pan

    i_pan = sub_pan_data.data


#--------------------------------------------------
#tilt入力関数
#--------------------------------------------------
def subf_tilt(sub_tilt_data):
    global i_tilt

    i_tilt = sub_tilt_data.data


#--------------------------------------------------
#pan出力関数
#--------------------------------------------------
def pubf_pan():
    global i_pan

    pub_pan = rospy.Publisher('/dy_servo_cam_pan/command', Float64, queue_size = 1)
    o_pan = Float64()
    o_pan = i_pan
    #可動範囲リミッタ
    if o_pan > MAX_LIM_PAN:
        o_pan = MAX_LIM_PAN
    if o_pan < MIN_LIM_PAN:
        o_pan = MIN_LIM_PAN
    pub_pan.publish(o_pan + OFFSET_PAN) #初期ズレ補正

    pan1_cam2pan2_cam = tf.TransformBroadcaster()
    pan1_cam2pan2_cam.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, o_pan), rospy.Time.now(), "pan2_cam", "pan1_cam")


#--------------------------------------------------
#tilt出力関数
#--------------------------------------------------
def pubf_tilt():
    global i_tilt

    pub_tilt = rospy.Publisher('/dy_servo_cam_tilt/command', Float64, queue_size = 1)
    o_tilt = Float64()
    o_tilt = i_tilt
    #可動範囲リミッタ
    if o_tilt > MAX_LIM_TILT:
        otilt = MAX_LIM_TILT
    if o_tilt < MIN_LIM_TILT:
        o_tilt = MIN_LIM_TILT
    pub_tilt.publish(-o_tilt + OFFSET_TILT) #初期ズレ補正 camとmicのpantiltユニットは逆に付いているので反転

    tilt1_cam2tilt2_cam = tf.TransformBroadcaster()
    tilt1_cam2tilt2_cam.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, o_tilt, 0), rospy.Time.now(), "tilt2_cam", "tilt1_cam")


#--------------------------------------------------
#main
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    rospy.sleep(5) #頻発するdynamixelパッケージのエラー対策

    rospy.Subscriber("/dy_servo_cam_pan/angle", Float64, subf_pan)
    rospy.Subscriber("/dy_servo_cam_tilt/angle", Float64, subf_tilt)

    main_rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        pubf_pan()
        pubf_tilt()
        main_rate.sleep()
