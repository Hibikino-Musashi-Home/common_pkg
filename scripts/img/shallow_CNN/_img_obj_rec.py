#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import cv2
import numpy as np
import skimage.data
import caffe
from caffe.proto import caffe_pb2

import actionlib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dev_arm_pkg.msg import ObjRecAction
from dev_arm_pkg.msg import ObjRecFeedback
from dev_arm_pkg.msg import ObjRecResult

import os
import sys

from subprocess import *

import time


#array for detected image
croped_width   = 128
croped_height  = 128

names = [
	'Green_tea/',           #0
   'Cafe_au_lait/',        #1
   'Iced_tea/',            #2
	'Orange_juice/',        #3
	'Strawberry_juice/',    #4
	'Potato_chips/',        #5
	'Cookie/',              #6
	'Potato_stick/',        #7
	'Potage_soup/',         #8
	'Egg_soup/',            #9
	'Orange/',              #10
	'Apple/'                #11
	,'Bawl/',               #12
	'Tray/',                #13
	'Cup/'                  #14
]

class ObjRec(object):
    def __init__(self):
        
        self._obj_rec_action_server = actionlib.SimpleActionServer('obj_rec_action', ObjRecAction, execute_cb = self.obj_rec)
        self._obj_rec_action_server.start()
        self._cv_bridge = CvBridge()
        
        #caffe classifier of shape
        learned_path    = os.path.abspath(os.path.dirname(__file__)) + '/160616_3CNN2MLP/'
        
        caffe.set_mode_gpu()
        prototxt_path   = learned_path + 'const.prototxt'
        caffemodel_path = learned_path + 'model.caffemodel.h5'
        mean_path       = learned_path + 'mean.binaryproto'

        mean_blob = caffe_pb2.BlobProto()
        with open(mean_path)as f:
            mean_blob.ParseFromString(f.read()) 
        mean_array = np.asarray(mean_blob.data, dtype = np.float32).reshape((mean_blob.channels, mean_blob.height, mean_blob.width))

        self.classifier = caffe.Classifier(prototxt_path, caffemodel_path, mean = mean_array, raw_scale = 255, channel_swap = (2, 1, 0))

    
    def obj_rec(self, goal):      
        start1 = time.time()      
        goal.obj_rec_goal.encoding = 'bgr8'

        #convert image
        cv2img = self._cv_bridge.imgmsg_to_cv2(goal.obj_rec_goal, 'bgr8')
        cv2img_array = np.array(cv2img, dtype = np.uint8)
        cv2.normalize(cv2img_array, cv2img_array, 0, 255, cv2.NORM_MINMAX)

        #check aspect propotion
        #aim_obj = rospy.get_param('/param/iarm/obj/id') - 1

        #resize image
        image = cv2.resize(cv2img_array, (croped_width, croped_height))  
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = skimage.img_as_float(image).astype(np.float32)

        start2 = time.time()        
        out   = self.classifier.predict([image], oversample = False)
        pred  = np.argmax(out)

        print 'time1: ' + str(time.time() - start1)
        print 'time2: ' + str(time.time() - start2)

        print names[pred]

        raw_input('#####Type enter key to start#####')

        result = ObjRecResult(obj_rec_result = pred + 1)
        self._obj_rec_action_server.set_succeeded(result)

    ''' 
    def obj_rec_dammy(self, img, label):
        image = cv2.resize(img, (croped_width, croped_height))  
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = skimage.img_as_float(image).astype(np.float32)

        out = self.classifier.predict([image], oversample = False)
        pred = np.argmax(out)

        print names[pred]
    '''

if __name__ == '__main__':
    
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])
    
    obj_rec = ObjRec()

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
