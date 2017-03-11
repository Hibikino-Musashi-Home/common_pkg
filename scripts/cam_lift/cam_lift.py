#! /usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#カメラリフトActionのROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/03
#
#memo:
#カメラリフトの長さ 0.555[m] 84(エンコーダ計測回数)
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


#--------------------------------------------------
#定数
#--------------------------------------------------
MAX_SPEED = 400 #リフト移動の最高速
STOP = 0 #リフト停止の速度


#--------------------------------------------------
#グローバル変数
#--------------------------------------------------
enc = 0 #エンコーダの値[m]
upper_sw = True #upperスイッチの状態 False: ON True: OFF
lower_sw = True #lowerスイッチの状態 False: ON True: OFF


#--------------------------------------------------
#エンコーダ値Subscribe関数
#--------------------------------------------------
def subf_enc(sub_enc):
    global enc
    enc = (float(sub_enc.data) / 84) * 0.555 #Arduinoのエンコーダ計測回数をエンコーダ値[m]に変換


#--------------------------------------------------
#upperスイッチの状態Subscribe関数
#--------------------------------------------------
def subf_upper_sw(sub_upper_sw):
    global upper_sw
    upper_sw = sub_upper_sw.data


#--------------------------------------------------
#upperスイッチの状態Subscribe関数
#--------------------------------------------------
def subf_lower_sw(sub_lower_sw):
    global lower_sw
    lower_sw = sub_lower_sw.data


#--------------------------------------------------
#カメラリフト操作Actionクラス
#--------------------------------------------------
class CamLift(object):
    def __init__(self):
        self._cam_lift_action_server = actionlib.SimpleActionServer('cam_lift_action', CamLiftAction, execute_cb = self.cam_lift)
        self._cam_lift_action_server.start()
        self._pub_speed = rospy.Publisher('/cam_lift/speed', Int64, queue_size = 1)


    def cam_lift(self, goal):
        global enc, upper_sw, lower_sw

        #upperまで移動
        if goal.cam_lift_goal == 0.555:
            self._pub_speed.publish(MAX_SPEED)
            while upper_sw == True:
                pass               
            self._pub_speed.publish(STOP)

            result = CamLiftResult(cam_lift_result = True)
            self._cam_lift_action_server.set_succeeded(result)
        #lowerまで移動
        elif goal.cam_lift_goal == 0: 
            self._pub_speed.publish(-MAX_SPEED)
            while lower_sw == True:
                    pass                
            self._pub_speed.publish(STOP)

            result = CamLiftResult(cam_lift_result = True)
            self._cam_lift_action_server.set_succeeded(result)
        elif goal.cam_lift_goal > 0 and goal.cam_lift_goal < 0.555:
            #上昇する時
            if goal.cam_lift_goal > enc:                
                self._pub_speed.publish(MAX_SPEED)
                while goal.cam_lift_goal > enc:
                    pass                
                self._pub_speed.publish(STOP)
            #下降する時
            elif goal.cam_lift_goal < enc:               
                self._pub_speed.publish(-MAX_SPEED)
                while goal.cam_lift_goal < enc:
                    pass             
                self._pub_speed.publish(STOP)

            result = CamLiftResult(cam_lift_result = True)
            self._cam_lift_action_server.set_succeeded(result)
        else:
            rospy.logwarn('[cam_lift]: 範囲外のカメラリフト位置が指定されました。')
            result = CamLiftResult(cam_lift_result = False)
            self._cam_lift_action_server.set_succeeded(result)


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    rospy.Subscriber("/cam_lift/enc", Int64, subf_enc)
    rospy.Subscriber("/cam_lift/upper_sw", Bool, subf_upper_sw)
    rospy.Subscriber("/cam_lift/lower_sw", Bool, subf_lower_sw)
        
    cam_lift = CamLift()

    lift1_cam2lift2_cam = tf.TransformBroadcaster()

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():    
        lift1_cam2lift2_cam.sendTransform((0, 0, enc),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "lift2_cam", "lift1_cam")
        main_rate.sleep()

