#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#人を発見するROSノード(台車動作なし)
#
#Hibikino-Musashi@Home
#author: Yutaro ISHIDA
#date: 16/03/17
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

NUM_ANGLE_LOOP_MAX = 200 #400

#フィールドサイズ設定部分
FIELD_MAX_X = 11.0
FIELD_MIN_X = 1.0
FIELD_MAX_Y = 10.0
FIELD_MIN_Y = -1.5


#------------------------------------------------
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
def findPerson():
    Id_tfTracking = 0

    Name_Tracking_head=""

    th = Move_Thread()

    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            if Id_tfTracking <= MAXID:
                Id_tfTracking += 1
                Name_Tracking_head = "head_" + str(Id_tfTracking)
            else:
                Id_tfTracking = 0
                Name_Tracking_head = "head"
            
        #    try:
        #        (map2headtrans,map2headrot)=tf_listener.lookupTransform("map", Name_Tracking_head, rospy.Time(0))
        #    except:
        #        continue
        #    break
        #
        #
        #while not rospy.is_shutdown():
            try:
                (camera2headtrans,camera2headrot)=tf_listener.lookupTransform("openni_depth_frame", Name_Tracking_head, rospy.Time(0))
            except:
                continue
            break
            

        if 1: #detectPersonInField(map2headtrans): #フィールド内にいる人か
            if camera2headtrans[0] <= DISTANCE_THERESHOLD: #判定距離以内にいる人か
                rospy.loginfo("DETECT_PERSON " + str(Id_tfTracking))
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                th.stop()
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                commonf_speech_multi('あなたを発見しました。私の前に近づいて下さい。')
                return 0


#------------------------------------------------
#------------------------------------------------
if __name__ == '__main__':    
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
        commonf_speech_multi('あなたを探します。')
    
        findPerson()
        sys.exit(0)
