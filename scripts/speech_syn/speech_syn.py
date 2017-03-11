#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#音声合成のROSノード
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
#音声合成ファイルを生成し再生する関数
#--------------------------------------------------
def syn_and_play(str_talk, lang):
    if lang == 'ja':
        args = [
            'open_jtalk',
            '-x', '/usr/local/dic',
            '-m', (os.path.abspath(os.path.dirname(__file__)) + '/MMDAgent_Example-1.4/Voice/mei/mei_normal.htsvoice'),
            '-ow', (os.path.abspath(os.path.dirname(__file__)) + '/output.wav')
        ]
    elif lang == 'en':
        args = [
            'flite_hts_engine',
            '-m', (os.path.abspath(os.path.dirname(__file__)) + '/hts_voice_cmu_us_arctic_slt-1.05/cmu_us_arctic_slt.htsvoice'),
            '-o', (os.path.abspath(os.path.dirname(__file__)) + '/output.wav')
        ]
    else:
        return 0

    #wavファイルの生成
    p = Popen(args, stdin=PIPE)
    p.communicate(str_talk)
    p.wait()

    #wavファイルを再生
    args = ['aplay', '-q', (os.path.abspath(os.path.dirname(__file__)) + '/output.wav')]
    p = Popen(args)
    p.wait()

    return 0


#--------------------------------------------------
#半角文字(英語)、全角文字(日本語)に分けて音声合成する
#--------------------------------------------------
def divide_half_full_and_syn_and_play(str_talk_full):
    #str文字列からunicode文字列に変換し、リストにする
    uni_char_list = list(str_talk_full.decode('utf-8'))

    str_ja = ''
    str_en = ''
    #unicode文字列リストを1文字ずつ取り出す
    for uni_char in uni_char_list:
        #全角文字の場合
        if ord(uni_char) > 255:
            str_ja += uni_char.encode('utf-8')
            last_lang = 'ja'
        #半角文字の場合
        else:
            str_en += uni_char.encode('utf-8')
            last_lang = 'en'

        #半角文字と全角文字が入れ替わった場合
        if len(str_ja) > 0 and len(str_en) > 0:
            #これまで半角文字ブロックを取り出していたら英語再生
            if last_lang == 'ja':
                syn_and_play(str_en, 'en')
                str_en = ''
            #これまで全角文字ブロックを取り出していたら日本語再生
            else:
                syn_and_play(str_ja, 'ja')
                str_ja = ''

    #最後の半角文字または全角文字ブロックの再生
    if len(str_ja) > 1:
        syn_and_play(str_ja, 'ja')
    else:
        syn_and_play(str_en, 'en')


#--------------------------------------------------
#音声合成Actionクラス
#--------------------------------------------------
class SpeechSyn(object):
    def __init__(self):
        self._speech_syn_action_server = actionlib.SimpleActionServer('speech_syn_action', SpeechSynAction, execute_cb = self.speech_syn)
        self._speech_syn_action_server.start()


    def speech_syn(self, goal):
        divide_half_full_and_syn_and_play(goal.speech_syn_goal)

        result = SpeechSynResult(speech_syn_result = True)
        self._speech_syn_action_server.set_succeeded(result)


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    #syn_and_play('This is a test message.', 'en')
    #syn_and_play('これはテストメッセージです。', 'ja')

    speech_syn = SpeechSyn()

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():    
        main_rate.sleep()
