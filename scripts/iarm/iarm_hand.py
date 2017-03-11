#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#iARM手渡しROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/03
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


#--------------------------------------------------
#定数
#--------------------------------------------------
IARM_HAND_POS = {'x':0.60, 'y':-0.10, 'z':1.00}
IARM_CLOSE_POS = {'x':0.20, 'y':0.18, 'z':0.75} #iARM格納時におけるエンドエフェクタ付け根のbase_linkに対する座標


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:        
        commonf_speech_multi('オブジェクトを手渡しします。')

        commonf_actionf_cam_lift(0.555)
        commonf_pubf_cam_pan(0.0)
        commonf_pubf_cam_tilt(0.0)

        tf_listener = tf.TransformListener()

        main_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            iarm_action_client = actionlib.SimpleActionClient('iarm_action', iARMAction)
            iarm_action_client.wait_for_server()

            goal = iARMGoal()


            goal.iarm_goal = 's:iARMCtrl,CarVel,0,0,0,0,0,-0.785:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            rospy.sleep(2)

            goal.iarm_goal = 's:iARMCtrl,CarVel,0,0,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #X移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,' + str(round((IARM_HAND_POS['x'] - IARM_CLOSE_POS['x']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #Z移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,0,' + str(round((IARM_HAND_POS['z'] - IARM_CLOSE_POS['z']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #Y移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,' + str(round(-(IARM_HAND_POS['y'] - IARM_CLOSE_POS['y']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #Z移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,0,' + str(round((IARM_HAND_POS['z'] - IARM_CLOSE_POS['z']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #Y移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,' + str(round(-(IARM_HAND_POS['y'] - IARM_CLOSE_POS['y']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #X移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,' + str(round((IARM_HAND_POS['x'] - IARM_CLOSE_POS['x']) * 1000 / 2, 0)) + ',0,0,0,0,0,200,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()


            commonf_speech_multi('オブジェクトを離します。受け取って下さい。')

            goal.iarm_goal = 's:iARMCtrl,EndOpening,1.0,0.2:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            goal.iarm_goal = 's:iARMCtrl,EndOpening,0,0.2:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()


            sys.exit(0)

            main_rate.sleep()