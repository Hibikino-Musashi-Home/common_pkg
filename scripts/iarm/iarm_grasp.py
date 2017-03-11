#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#iARMオブジェクト把持ROSノード
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
IARM_INIT_POS = {'x':0.482, 'y':-0.451, 'z':1.03} #iARM展開時におけるエンドエフェクタ付け根のbase_linkに対する座標


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:  
        commonf_speech_multi('オブジェクトを掴みます。')

        commonf_actionf_cam_lift(0.555)
        commonf_pubf_cam_pan(-0.348)
        commonf_pubf_cam_tilt(0.785)
      
        iarm_obj_pos = {'x':0.0, 'y':0.0, 'z':0.0}
        iarm_obj_pos['x'] = rospy.get_param('/param/iarm/obj/pos/x')
        iarm_obj_pos['y'] = rospy.get_param('/param/iarm/obj/pos/y') - 0.1
        iarm_obj_pos['z'] = rospy.get_param('/param/iarm/obj/pos/z')

        tf_listener = tf.TransformListener()

        main_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            iarm_action_client = actionlib.SimpleActionClient('iarm_action', iARMAction)
            iarm_action_client.wait_for_server()

            goal = iARMGoal()

            #Y移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,' + str(round(-(iarm_obj_pos['y'] - IARM_INIT_POS['y'] - 0.1) * 1000, 0)) + ',0,0,0,0,0,300,0,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #X移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,' + str(round((iarm_obj_pos['x'] - IARM_INIT_POS['x']) * 1000, 0)) + ',0,0,0,0,0,300,0,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()

            #Z移動
            goal.iarm_goal = 's:iARMCtrl,CarDis,0,0,' + str(round((iarm_obj_pos['z'] - IARM_INIT_POS['z'] + 0.05) * 1000, 0)) + ',0,0,0,0,0,300,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()


            end_vel = {'x':1.0, 'y':1.0, 'z':1.0}

            while end_vel['x'] != 0 or end_vel['y'] != 0 or end_vel['z'] != 0:
                try:
                    (translation, rotation) = tf_listener.lookupTransform('base_link', 'ar_marker_8', rospy.Time(0))
                except:
                    continue

                end_vel['x'] = 0
                end_vel['y'] = 0
                end_vel['z'] = 0

                if abs(iarm_obj_pos['x'] - translation[0]) > 0.01:
                    if iarm_obj_pos['x'] < translation[0]:
                        #end_vel['y'] = -15
                        print 'hoge'
                    else:
                        #end_vel['y'] = 15
                        print 'piyo'
                if abs(iarm_obj_pos['y'] - translation[1]) > 0.01:
                    if iarm_obj_pos['y'] < translation[1]:
                        end_vel['x'] = 15
                    else:
                        end_vel['x'] = -15

                goal.iarm_goal = 's:iARMCtrl,CarVel,' + str(end_vel['x']) + ',' + str(end_vel['y']) + ',' + str(end_vel['z']) + ',0,0,0:e'

                iarm_action_client.send_goal(goal)
                iarm_action_client.wait_for_result()

                if rospy.is_shutdown():
                    goal.iarm_goal = 's:iARMCtrl,KeyCmd,27:e'
                    iarm_action_client.send_goal(goal)
                    iarm_action_client.wait_for_result()
                    sys.exit()


            goal.iarm_goal = 's:iARMCtrl,EndOpening,0.5,0.2:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()


            goal.iarm_goal = 's:iARMCtrl,CarDis,0,0,100,0,0,0,0,0,300,0,0,0:e'
            iarm_action_client.send_goal(goal)
            iarm_action_client.wait_for_result()


            commonf_speech_multi('オブジェクトを掴みました。')

            sys.exit(0)

            main_rate.sleep()
