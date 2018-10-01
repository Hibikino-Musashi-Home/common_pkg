#!/usr/bin/env python
# -*- coding: utf-8 -*-


#==================================================
#音声認識ROSノード
#
#author: Yuki KURODA
#==================================================


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/script/common')

from common_import import *
from common_function import CommonFunction


#==================================================
#パラメータ
#==================================================
GP_LOOP_RATE = 1

GP_MIC_NO = 0
GP_MIC_VOL = 15

GP_KEY_PHRASE = 'hey you'
GP_TH_KEY_PHRASE = 1e-20
#hey thomas -25~-30
#hey you -20


#==================================================
#グローバル
#==================================================


#==================================================
#音声認識クラス
#==================================================
class SpeechRec:
    #==================================================
    #コンストラクタ
    #==================================================
    def __init__(self):
        #--------------------------------------------------
        #インスタンス変数
        #--------------------------------------------------
        self._cf = CommonFunction()


        #--------------------------------------------------
        #ROSパラメータ
        #--------------------------------------------------
        self._p_loop_rate = self._cf.get_ros_param_this('/p/config/loop_rate', GP_LOOP_RATE)

        self._p_mic_no = self._cf.get_ros_param_this('/p/config/mic_no', GP_MIC_NO)
        self._p_mic_vol = self._cf.get_ros_param_this('/p/config/mic_vol', GP_MIC_VOL)

        self._p_model_dir = self._cf.get_ros_param_this('/p/config/model_dir', os.path.expanduser('~'))
        self._p_gram_dir = self._cf.get_ros_param_this('/p/config/gram_dir', os.path.expanduser('~'))
        self._p_hmm = self._cf.get_ros_param_this('/p/config/hmm', 'en-us/en-us')
        self._p_dict = self._cf.get_ros_param_this('/p/config/hmm', 'en-us/cmudict-en-us.dict')
        self._p_task = self._cf.get_ros_param_this('/p/config/task', '')


        #--------------------------------------------------
        #ROSインタフェース
        #--------------------------------------------------
        self._as_speech_rec = actionlib.SimpleActionServer(self._cf.get_ros_interface_name_this('/a/speech_rec'), SpeechRecAction, execute_cb = self.af_speech_rec)
        self._as_speech_rec.start()


    #==================================================
    #デストラクタ
    #==================================================
    def __del__(self):
        pass


    #==================================================
    #音声認識アクション関数
    #==================================================
    def af_speech_rec(self, goal):
        #self._decoder_1.start_utt()
        #self._stream.start_stream()

        #音声認識開始音
        #self._cf.ctrl_a_sound_effect(1)
        self._cf.ctrl_t_sound_effect(1)
        rospy.sleep(0.5)

        #self._decoder_1.end_utt()
        #self._stream.stop_stream()

        os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" ' + str(self._p_mic_vol) + '%')

        self._decoder_1.start_utt()
        self._stream.start_stream()

        rospy.loginfo('[' + rospy.get_name() + ']: ---------- Recognize key phrase ----------')

        while not rospy.is_shutdown():
            buf = self._stream.read(1024)
            if buf:
                self._decoder_1.process_raw(buf, False, False)
            else:
                os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" 0%')
                self._stream.stop_stream()
                rospy.logwarn('[' + rospy.get_name() + ']: Recognize FAILURE')
                result = SpeechRecResult()
                result.result.data = ''
                self._as_speech_rec.set_succeeded(result)
                return

            if self._decoder_1.hyp() is not None:
                self._decoder_1.end_utt()
                for seg in self._decoder_1.seg():
                    if GP_KEY_PHRASE in seg.word:
                        in_speech_bf = False

                        rospy.loginfo('[' + rospy.get_name() + ']: ---------- Recognize speech ----------')
                        self._decoder_2.start_utt()

                        while not rospy.is_shutdown():
                            buf = self._stream.read(1024)
                            if buf:
                                self._decoder_2.process_raw(buf, False, False)
                            else:
                                os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" 0%')
                                self._stream.stop_stream()
                                rospy.logwarn('[' + rospy.get_name() + ']: Recognize FAILURE')
                                result = SpeechRecResult()
                                result.result.data = ''
                                self._as_speech_rec.set_succeeded(result)
                                return

                            if self._decoder_2.get_in_speech() != in_speech_bf:
                                in_speech_bf = self._decoder_2.get_in_speech()
                                if not in_speech_bf:
                                    self._decoder_2.end_utt()
                                    if self._decoder_2.hyp() is not None:
                                        os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" 0%')
                                        self._stream.stop_stream()
                                        rospy.loginfo('[' + rospy.get_name() + ']: ' + self._decoder_2.hyp().hypstr)
                                        result = SpeechRecResult()
                                        result.result.data = self._decoder_2.hyp().hypstr
                                        self._as_speech_rec.set_succeeded(result)
                                        return

                                    os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" 0%')
                                    self._stream.stop_stream()
                                    rospy.logwarn('[' + rospy.get_name() + ']: Recognize FAILURE')
                                    result = SpeechRecResult()
                                    result.result.data = ''
                                    self._as_speech_rec.set_succeeded(result)
                                    return


    #==================================================
    #クラス初期化関数
    #==================================================
    def class_init(self):
        os.system('amixer -c ' + str(self._p_mic_no) + ' sset "Mic" 0%')

        config_1 = Decoder.default_config()
        config_1.set_string('-hmm', os.path.join(self._p_model_dir, self._p_hmm))
        config_1.set_string('-dict', os.path.join(self._p_model_dir, self._p_dict))
        config_1.set_string('-keyphrase', GP_KEY_PHRASE)
        config_1.set_float('-kws_threshold', GP_TH_KEY_PHRASE)

        config_2 = Decoder.default_config()
        config_2.set_string('-hmm', os.path.join(self._p_model_dir, self._p_hmm))
        config_2.set_string('-dict', os.path.join(self._p_model_dir, self._p_dict))
        print '----------hogehogehogehoge----------'        
        pa = pyaudio.PyAudio()
        self._stream = pa.open(format = pyaudio.paInt16, channels = 1, rate = 16000, input = True, frames_per_buffer = 1024)
        #self._stream.start_stream()
        print '----------hogehogehogehoge----------'        
        self._decoder_1 = Decoder(config_1)
        self._decoder_2 = Decoder(config_2)

        jsgf = Jsgf(os.path.join(self._p_gram_dir, self._p_task + '.gram'))
        rule = jsgf.get_rule(self._p_task + '.common')
        fsg = jsgf.build_fsg(rule, self._decoder_2.get_logmath(), 7.5)
        fsg.writefile(self._p_task + '.fsg')

        self._decoder_2.set_fsg(self._p_task, fsg)
        self._decoder_2.set_search(self._p_task)


    #==================================================
    #クラスループ関数
    #==================================================
    def class_loop(self):
        loop_wait = rospy.Rate(self._p_loop_rate)
        while not rospy.is_shutdown():
            self.class_main()

            if self._p_loop_rate != self._cf.get_ros_param_this('/p/config/loop_rate', GP_LOOP_RATE):
                self._p_loop_rate = self._cf.get_ros_param_this('/p/config/loop_rate', GP_LOOP_RATE)
                loop_wait = rospy.Rate(self._p_loop_rate)

            loop_wait.sleep()


    #==================================================
    #クラスメイン関数
    #==================================================
    def class_main(self):
        pass


#==================================================
#メイン関数
#==================================================
if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0])

    speech_rec = SpeechRec()
    speech_rec.class_init()
    speech_rec.class_loop()

    loop_wait = rospy.Rate(GP_LOOP_RATE)
    while not rospy.is_shutdown():
        loop_wait.sleep()
