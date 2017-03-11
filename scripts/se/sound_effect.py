#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#効果音再生のROSノード
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
#効果音再生Actionクラス
#--------------------------------------------------
class SoundEffect(object):
    def __init__(self):
        self._sound_effect_action_server = actionlib.SimpleActionServer('sound_effect_action', SoundEffectAction, execute_cb = self.sound_effect)
        self._sound_effect_action_server.start()


    def sound_effect(self, goal):
        if goal.sound_effect_goal == 'launch':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/launch.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        elif goal.sound_effect_goal == 'speech_rec':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/speech_rec.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        elif goal.sound_effect_goal == 'follow':
            call(['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/follow.wav')])
            result = SoundEffectResult(sound_effect_result = True)
            self._sound_effect_action_server.set_succeeded(result)
        else:
            rospy.logwarn('[sound_effect]: 対応する効果音がありません。')
            result = SoundEffectResult(sound_effect_result = False)
            self._sound_effect_action_server.set_succeeded(result)


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    sound_effect = SoundEffect()

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():    
        main_rate.sleep()
