#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#ROS関係(import)
#--------------------------------------------------
import rospy
import rosbag

import tf

import smach
import smach_ros

import actionlib


#--------------------------------------------------
#ROS関係(from import)
#--------------------------------------------------
from cv_bridge import CvBridge

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

from common_pkg.msg import SpeechSynAction
from common_pkg.msg import SpeechSynGoal
from common_pkg.msg import SpeechSynFeedback
from common_pkg.msg import SpeechSynResult

from common_pkg.msg import SpeechRecAction
from common_pkg.msg import SpeechRecGoal
from common_pkg.msg import SpeechRecFeedback
from common_pkg.msg import SpeechRecResult

from common_pkg.msg import SoundEffectAction
from common_pkg.msg import SoundEffectGoal
from common_pkg.msg import SoundEffectFeedback
from common_pkg.msg import SoundEffectResult

from common_pkg.msg import ObjRecAction
from common_pkg.msg import ObjRecGoal
from common_pkg.msg import ObjRecFeedback
from common_pkg.msg import ObjRecResult

from common_pkg.msg import CamLiftAction
from common_pkg.msg import CamLiftGoal
from common_pkg.msg import CamLiftFeedback
from common_pkg.msg import CamLiftResult

from common_pkg.msg import iARMAction
from common_pkg.msg import iARMGoal
from common_pkg.msg import iARMFeedback
from common_pkg.msg import iARMResult

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


#--------------------------------------------------
#システム関係(import)
#--------------------------------------------------
import sys
import os
import termios
import tty
import select
import time
import datetime
import threading
import math
import re
import socket

import pygame #ゲームパッドを使うためのライブラリ


#--------------------------------------------------
#システム関係(from import)
#--------------------------------------------------
from subprocess import call
from subprocess import Popen
from subprocess import PIPE

from pygame.locals import * #pygame.localsの定数群を使うため
