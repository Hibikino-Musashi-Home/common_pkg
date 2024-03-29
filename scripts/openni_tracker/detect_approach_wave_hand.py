#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#腕を振っている人を検出するプログラム
#
#Hibikino-Musashi@Home
#author: Yuta KIYAMA
#date: 16/03/14 
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


#------------------------------------------------
#定数
#------------------------------------------------
#定数の読み込みはglobal宣言なしで問題なし

MAXID = 9 #トラッキングする最大人数
DISTANCE_THERESHOLD = 4.0 #判定距離

NUM_ANGLE_LOOP_MAX = 44 #200 45[deg] #400 90[deg]

#フィールドサイズ設定部分
FIELD_MAX_X = 20
FIELD_MIN_X = -20
FIELD_MAX_Y = 20
FIELD_MIN_Y = -20

HEAD_THRESHOLD = 0.5 #止まっている人と認識する閾値　この値より低いと止まっている人
ELBOW_THRESHOLD = 0.087 #肘が回転しているかの閾値　この値より高いと肘が回転していると認識
HAND_THRESHOLD = 0.10 #手先が動いているかの閾値　この値より高いと手先が動いていると認識

WAVE_JUDGE_NUM = 3 #連続何回認識したらプログラムを終了するか
NEXT_JUDGE_NUM = -5


#------------------------------------------------
#グローバル変数
#------------------------------------------------
last_head_l_x = 0
last_head_l_y = 0
last_elbow_l_pitch = 0
last_hand_l_z = 0
last_head_r_x = 0
last_head_r_y = 0
last_elbow_r_pitch = 0
last_hand_r_z = 0


#------------------------------------------------
#------------------------------------------------
def cal_yaw(translationA, translationB):
   x1 = translationA[0]
   y1 = translationA[1]
   x2 = translationB[0]
   y2 = translationB[1]

   if x2 - x1 == 0:
        x2 += 0.00000001 #0による除算を回避するための処置
  
   if x2 - x1 >= 0 and y2 - y1 >= 0:
        return math.atan((y2 - y1) / (x2 - x1))
   if x2 - x1 <= 0 and y2 - y1 >= 0:
        return math.atan((y2 - y1) / (x2 - x1)) + math.pi
   if x2 - x1 <= 0 and y2 - y1 <= 0:
        return math.atan((y2 - y1) / (x2 - x1)) - math.pi
   if x2 - x1 >= 0 and y2 - y1 < 0:
        return math.atan((y2 - y1) / (x2 - x1))


#------------------------------------------------
#頭座標からその人がフィールド内にいるかを判定する関数
#------------------------------------------------
def detectPersonInField(tf_head_translation):
    #パラメータサーバーからフィールドの範囲を取得
    #FIELD_MAX_X = n.getParam("/navigation/field/MAX_X");
    #FIELD_MIN_X = n.getParam("/navigation/field/MIN_X");
    #FIELD_MAX_Y = n.getParam("/navigation/field/MAX_Y");
    #FIELD_MIN_Y = n.getParam("/navigation/field/MIN_Y");

    #判定部分
    if tf_head_translation[0] <= FIELD_MAX_X and \
       tf_head_translation[0] >= FIELD_MIN_X and \
       tf_head_translation[1] <= FIELD_MAX_Y and \
       tf_head_translation[1] >= FIELD_MIN_Y:
        return 1 #フィールド内にいる
    else:
        return 0 #フィールド内にいない


#------------------------------------------------
#台車を左右に振るスレッド
#1段階目 台車 前->左
#2段階目 台車 左->前
#3段階目 台車 前->右
#4段階目 台車 右->前 1段階目に戻る
#------------------------------------------------
class Move_Thread():
    def __init__(self):
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target = self.target)
        self.thread.setDaemon(True)
        self.thread.start()

    def target(self):
        num_angle_loop = 0
        while not self.stop_event.is_set():            
            if num_angle_loop == 0 or num_angle_loop == NUM_ANGLE_LOOP_MAX / 4 * 4:
                num_angle_loop = 0
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.15) #1段階目
            elif num_angle_loop == NUM_ANGLE_LOOP_MAX / 4 * 1:
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.15) #2段階目
            elif num_angle_loop == NUM_ANGLE_LOOP_MAX / 4 * 2:
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0.15) #3段階目
            elif num_angle_loop == NUM_ANGLE_LOOP_MAX / 4 * 3:
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, -0.15) #4段階目
            num_angle_loop += 1
            rospy.sleep(0.1)

    def stop(self): #停止時処理
        commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
        self.stop_event.set()
        self.thread.join()


#------------------------------------------------
#------------------------------------------------
def detectWaveHand_L(translation_head, rotation_elbow, translation_hand):
    global last_head_l_x, last_head_l_y, last_elbow_l_pitch, last_hand_l_z

    euler = euler_from_quaternion([rotation_elbow[0], rotation_elbow[1], rotation_elbow[2], rotation_elbow[3]])
    
    #rospy.loginfo('Movement of head l x: ' + str(abs(last_head_l_x - translation_head[0])))
    #rospy.loginfo('Movement of head l y: ' + str(abs(last_head_l_y - translation_head[1])))
    #rospy.loginfo('Movement of elbow l pitch: ' + str(abs(last_elbow_l_pitch - euler[1])))
    #rospy.loginfo('Movement of hand l z: ' + str(abs(last_hand_l_z - translation_hand[2])))

    if abs(last_head_l_x - translation_head[0]) < HEAD_THRESHOLD and abs(last_head_l_y - translation_head[1]) < HEAD_THRESHOLD:
        if abs(last_elbow_l_pitch - euler[1]) > ELBOW_THRESHOLD:
            if abs(last_hand_l_z - translation_hand[2]) > HAND_THRESHOLD:
                last_head_l_x = translation_head[0]
                last_head_l_y = translation_head[1]
                last_elbow_l_pitch = euler[1]
                last_hand_l_z = translation_hand[2]
                rospy.loginfo('Waving arm')
                return 1

    last_head_l_x = translation_head[0]
    last_head_l_y = translation_head[1]
    last_elbow_l_pitch = euler[1]
    last_hand_l_z = translation_hand[2]
    return 0


#------------------------------------------------
#------------------------------------------------
def detectWaveHand_R(translation_head, rotation_elbow, translation_hand):
    global last_head_r_x, last_head_r_y, last_elbow_r_pitch, last_hand_r_z

    euler = euler_from_quaternion([rotation_elbow[0], rotation_elbow[1], rotation_elbow[2], rotation_elbow[3]])
    
    #rospy.loginfo('Movement of head r x: ' + str(abs(last_head_r_x - translation_head[0])))
    #rospy.loginfo('Movement of head r y: ' + str(abs(last_head_r_y - translation_head[1])))
    #rospy.loginfo('Movement of elbow r pitch: ' + str(abs(last_elbow_r_pitch - euler[1])))
    #rospy.loginfo('Movement of hand r z: ' + str(abs(last_hand_r_z - translation_hand[2])))

    if abs(last_head_r_x - translation_head[0]) < HEAD_THRESHOLD and abs(last_head_r_y - translation_head[1]) < HEAD_THRESHOLD:
        if abs(last_elbow_r_pitch - euler[1]) > ELBOW_THRESHOLD:
            if abs(last_hand_r_z - translation_hand[2]) > HAND_THRESHOLD:
                last_head_r_x = translation_head[0]
                last_head_r_y = translation_head[1]
                last_elbow_r_pitch = euler[1]
                last_hand_r_z = translation_hand[2]
                rospy.loginfo('Waving arm')
                return 1

    last_head_r_x = translation_head[0]
    last_head_r_y = translation_head[1]
    last_elbow_r_pitch = euler[1]
    last_hand_r_z = translation_hand[2]
    return 0


#------------------------------------------------
#------------------------------------------------
if __name__ == '__main__':    
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:        
        commonf_speech_multi('こちらを向いて手を振ってください。')


        tfTrackingId = 0

        judge_wave = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        th = Move_Thread()

        tf_listener = tf.TransformListener()

        main_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                if tfTrackingId < MAXID:
                    tfTrackingId += 1
                    Name_Tracking_head = 'head_'+str(tfTrackingId)
                    Name_Tracking_elbow_L = 'left_elbow_'+str(tfTrackingId)
                    Name_Tracking_elbow_R = 'right_elbow_'+str(tfTrackingId)
                    Name_Tracking_hand_L = 'left_hand_'+str(tfTrackingId)
                    Name_Tracking_hand_R = 'right_hand_'+str(tfTrackingId)
                else:
                    tfTrackingId = 0
                    Name_Tracking_head = 'head'
                    Name_Tracking_elbow_L = 'left_elbow'
                    Name_Tracking_elbow_R = 'right_elbow'
                    Name_Tracking_hand_L = 'left_hand'
                    Name_Tracking_hand_R = 'right_hand'
                
                try:
                    (camera2head_translation, camera2head_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_head, rospy.Time(0))
                    (camera2elbow_l_translation, camera2elbow_l_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_elbow_L, rospy.Time(0))
                    (camera2elbow_r_translation, camera2elbow_r_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_elbow_R, rospy.Time(0))
                    (camera2hand_l_translation, camera2hand_l_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_hand_L, rospy.Time(0))
                    (camera2hand_r_translation, camera2hand_r_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_hand_R, rospy.Time(0))
                except Exception as e:
                    #rospy.loginfo('type:'+str(type(e)))
                    #rospy.loginfo('args:'+str(e.args))
                    #rospy.loginfo('message:'+e.message)
                    #rospy.sleep(1)
                    continue
                break


            if 1: #detectPersonInField(camera2head_translation): #フィールド内にいる人か(必ず map x: 0 y: 0 にロボットが居ること)
                if camera2head_translation[0] <= DISTANCE_THERESHOLD: #判定距離内にいる人か
                    while not rospy.is_shutdown():
                        try:
                            (camera2head_translation, camera2head_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_head, rospy.Time(0))
                            (camera2elbow_l_translation, camera2elbow_l_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_elbow_L, rospy.Time(0))
                            (camera2elbow_r_translation, camera2elbow_r_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_elbow_R, rospy.Time(0))
                            (camera2hand_l_translation, camera2hand_l_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_hand_L, rospy.Time(0))
                            (camera2hand_r_translation, camera2hand_r_rotation) = tf_listener.lookupTransform('openni_depth_frame', Name_Tracking_hand_R, rospy.Time(0))
                            (map2head_translation, map2head_rotation) = tf_listener.lookupTransform('map', Name_Tracking_head, rospy.Time(0))
                        except Exception as e:
                            break


                        if detectWaveHand_L(camera2head_translation, camera2elbow_l_rotation, camera2hand_l_translation) or \
                           detectWaveHand_R(camera2head_translation, camera2elbow_r_rotation, camera2hand_r_translation):
                            if judge_wave[tfTrackingId] < 0:
                                judge_wave[tfTrackingId] = 0
                            judge_wave[tfTrackingId] += 1
                        else:
                            if judge_wave[tfTrackingId] > 0:
                                judge_wave[tfTrackingId] = 0
                            judge_wave[tfTrackingId] -= 1


                        if judge_wave[tfTrackingId] >= WAVE_JUDGE_NUM:
                            th.stop()
                            commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

                            commonf_speech_multi('あなたを発見しました。近づきます。')

                            while not rospy.is_shutdown():
                                try:
                                    (map2base_translation, map2base_rotation) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                                except Exception as e:
                                    continue
                    
                                base2head_yaw = cal_yaw(map2base_translation ,map2head_translation)
                                commonf_actionf_move_base(map2head_translation[0] - 1.2 * math.cos(base2head_yaw),
                                                          map2head_translation[1] - 1.2 * math.sin(base2head_yaw),
                                                          base2head_yaw)
                                commonf_pubf_cmd_vel(0.2, 0, 0, 0, 0, 0)
                                rospy.sleep(2)
                                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

                                sys.exit(0)


                        if judge_wave[tfTrackingId] <= NEXT_JUDGE_NUM:
                            break                    


                        main_rate.sleep()
