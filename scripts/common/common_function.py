#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts')

from common_import import *


#--------------------------------------------------
#グローバル変数
#--------------------------------------------------
settings = termios.tcgetattr(sys.stdin)

pub_scan_mode = rospy.Publisher('/scan/mode', String, queue_size = 1)
pub_cam_pan_angle = rospy.Publisher('/dy_servo_cam_pan/angle', Float64, queue_size = 1)
pub_cam_tilt_angle = rospy.Publisher('/dy_servo_cam_tilt/angle', Float64, queue_size = 1)
pub_mic_pan_angle = rospy.Publisher('/dy_servo_mic_pan/angle', Float64, queue_size = 1)
pub_mic_tilt_angle = rospy.Publisher('/dy_servo_mic_tilt/angle', Float64, queue_size = 1)
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)


#--------------------------------------------------
#音声合成を現スレッドで起動(喋り終わるまで待つ)
#半角文字は英語、全角文字は日本語で喋る
#例: speech_single('This is English test message.これは日本語のテストメッセージです。')
#--------------------------------------------------
#TODO:そのうちactionfを名前に入れる
def commonf_speech_single(speech_str):
    if rospy.get_param('/param/dbg/speech/ssynlog') == 1:
        rospy.loginfo(speech_str)
    if rospy.get_param('/param/dbg/sm/flow') == 0:
        speech_syn_action_client = actionlib.SimpleActionClient('speech_syn_action', SpeechSynAction)
        speech_syn_action_client.wait_for_server()
        
        goal = SpeechSynGoal()
        goal.speech_syn_goal = speech_str
        speech_syn_action_client.send_goal(goal)
        speech_syn_action_client.wait_for_result()


#--------------------------------------------------
#音声合成を他スレッドで起動
#半角文字は英語、全角文字は日本語で喋る
#例: speech_single('This is English test message.これは日本語のテストメッセージです。')
#--------------------------------------------------
#TODO:そのうちactionfを名前に入れる
def commonf_speech_multi(speech_str):
    if rospy.get_param('/param/dbg/speech/ssynlog') == 1:
        rospy.loginfo(speech_str)
    if rospy.get_param('/param/dbg/sm/flow') == 0:
        speech_syn_action_client = actionlib.SimpleActionClient('speech_syn_action', SpeechSynAction)
        speech_syn_action_client.wait_for_server()
        
        goal = SpeechSynGoal()
        goal.speech_syn_goal = speech_str
        speech_syn_action_client.send_goal(goal)
        #speech_syn_action_client.wait_for_result()


#--------------------------------------------------
#--------------------------------------------------
def commonf_actionf_sound_effect_single(sound_effect_goal):
    if rospy.get_param('/param/dbg/sm/flow') == 0:
        sound_effect_action_client = actionlib.SimpleActionClient('sound_effect_action', SoundEffectAction)
        sound_effect_action_client.wait_for_server()
        
        goal = SoundEffectGoal()
        goal.sound_effect_goal = sound_effect_goal
        sound_effect_action_client.send_goal(goal)
        sound_effect_action_client.wait_for_result()


#--------------------------------------------------
#--------------------------------------------------
def commonf_actionf_sound_effect_multi(sound_effect_goal):
    if rospy.get_param('/param/dbg/sm/flow') == 0:
        sound_effect_action_client = actionlib.SimpleActionClient('sound_effect_action', SoundEffectAction)
        sound_effect_action_client.wait_for_server()
        
        goal = SoundEffectGoal()
        goal.sound_effect_goal = sound_effect_goal
        sound_effect_action_client.send_goal(goal)
        #sound_effect_action_client.wait_for_result()


#--------------------------------------------------
#--------------------------------------------------
def commonf_node_killer(node_name): 
    node_killer_p = Popen(['rosnode','list'], stdout=PIPE) 
    node_killer_p.wait() 
    node_list = node_killer_p.communicate() 
    n = node_list[0] 
    n = n.split("\n") 
    for i in range(len(n)): 
        tmp = n[i]   
        if tmp.find(node_name) == 1: 
            call(['rosnode', 'kill', n[i]]) 
            break


#--------------------------------------------------
#--------------------------------------------------
def commonf_dbg_sm_stepin():
    if rospy.get_param('/param/dbg/sm/stepin') == 1:
        raw_input('#####Type enter key to step in#####')


#--------------------------------------------------
#--------------------------------------------------
def commonf_dbg_sm_stepout():
    if rospy.get_param('/param/dbg/sm/stepout') == 1:
        raw_input('#####Type enter key to step out#####')


#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_scan_mode(scan_mode):
    global pub_scan_mode #= rospy.Publisher('/scan/mode', String, queue_size = 1)
    pub_scan_mode.publish(scan_mode)
    rospy.sleep(0.05)
    pub_scan_mode.publish(scan_mode)


#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_cam_pan(cam_pan_angle):
    global pub_cam_pan_angle #= rospy.Publisher('/dy_servo_cam_pan/angle', Float64, queue_size = 1)
    pub_cam_pan_angle.publish(cam_pan_angle)
    rospy.sleep(0.05)
    pub_cam_pan_angle.publish(cam_pan_angle)


#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_cam_tilt(cam_tilt_angle):
    global pub_cam_tilt_angle #= rospy.Publisher('/dy_servo_cam_tilt/angle', Float64, queue_size = 1)
    pub_cam_tilt_angle.publish(cam_tilt_angle)
    rospy.sleep(0.05)
    pub_cam_tilt_angle.publish(cam_tilt_angle)


#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_mic_pan(mic_pan_angle):
    global pub_mic_pan_angle #= rospy.Publisher('/dy_servo_mic_pan/angle', Float64, queue_size = 1)
    pub_mic_pan_angle.publish(mic_pan_angle)
    rospy.sleep(0.05)    
    pub_mic_pan_angle.publish(mic_pan_angle)
    

#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_mic_tilt(mic_tilt_angle):
    global pub_mic_tilt_angle #= rospy.Publisher('/dy_servo_mic_tilt/angle', Float64, queue_size = 1)
    pub_mic_tilt_angle.publish(mic_tilt_angle)
    rospy.sleep(0.05)
    pub_mic_tilt_angle.publish(mic_tilt_angle)
    

#--------------------------------------------------
#--------------------------------------------------
def commonf_pubf_cmd_vel(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    global pub_cmd_vel #= rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_x
    cmd_vel.linear.y = linear_y
    cmd_vel.linear.z = linear_z
    cmd_vel.angular.x = angular_x
    cmd_vel.angular.y = angular_y
    cmd_vel.angular.z = angular_z
    pub_cmd_vel.publish(cmd_vel)
    rospy.sleep(0.05)
    pub_cmd_vel.publish(cmd_vel)


#--------------------------------------------------
#--------------------------------------------------
def commonf_actionf_cam_lift(cam_lift_goal):
    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
        cam_lift_action_client = actionlib.SimpleActionClient('cam_lift_action', CamLiftAction)
        cam_lift_action_client.wait_for_server()
        
        goal = CamLiftGoal()
        goal.cam_lift_goal = cam_lift_goal
        cam_lift_action_client.send_goal(goal)
        cam_lift_action_client.wait_for_result()


#--------------------------------------------------
#--------------------------------------------------
def commonf_actionf_speech_rec(speech_rec_goal):
    speech_rec_action_client = actionlib.SimpleActionClient('speech_rec_action', SpeechRecAction) #音声認識のActionClientを生成
    speech_rec_action_client.wait_for_server() #音声認識ノードのActionServerに接続

    goal = SpeechRecGoal() #ActionのGoalを生成
    goal.speech_rec_goal = speech_rec_goal #ActionのGoalを設定
    speech_rec_action_client.send_goal(goal) #音声認識ノードのActionServerにGoalを送信

    speech_rec_action_client.wait_for_result() #音声認識ノードのActionServerから終了が返って来るまで待つ
    return speech_rec_action_client.get_result().speech_rec_result


#--------------------------------------------------
#--------------------------------------------------
def commonf_actionf_move_base(x, y, yaw):
    if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:
        rospy.loginfo('Goal pos x: ' + str(x) + ' y: ' + str(y) + ' yaw: ' + str(yaw))

        move_base_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        move_base_action_client.wait_for_server()

        quaternion = quaternion_from_euler(0.0, 0.0, yaw)        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        move_base_action_client.send_goal(goal)

        #move_base_action_client.wait_for_result()

        rospy.set_param('/param/slam/goal/pos/x', float(x)) #float
        rospy.set_param('/param/slam/goal/pos/y', float(y)) #float
        rospy.set_param('/param/slam/goal/pos/yaw', float(yaw)) #float
                
        call(['rosrun', 'common_pkg', 'slam_goto.py'])

        move_base_action_client.cancel_goal()


#--------------------------------------------------
#--------------------------------------------------
def commonf_get_key():
    global settings

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key
