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
	['Green_tea/', 'Cafe_au_lait/', 'Iced_tea/'],
	['Orange_juice/', 'Strawberry_juice/'],
	['Potato_chips/', 'Potato_stick/', 'Potage_soup/', 'Egg_soup/', 'Cup/'],
    ['Cookie_0/', 'Cookie_1/', 'Cookie_2/', 'Cookie_3/', 'Cookie_white/', 'Cookie_yellow/'],
    ['Orange/', 'Apple/', 'Bawl/', 'Tray/']
]

class ObjRec(object):
    def __init__(self):
        caffe.set_mode_gpu()

        self._obj_rec_action_server = actionlib.SimpleActionServer('obj_rec_action', ObjRecAction, execute_cb = self.obj_rec)
        self._obj_rec_action_server.start()
        self._cv_bridge = CvBridge()
        
        #caffe classifier of shape
        learned_path    = os.path.abspath(os.path.dirname(__file__)) + '/160616_HOG_3CNN2MLP/'

        #set HOG parameters
        self.hog = cv2.HOGDescriptor(
		        (128, 128),	#window size
		        (32, 32),	#block size
		        (32, 32),	#block stride
		        (32, 32),	#cell size
		        9			#bin
        )
	
        #load svm
        self.svm = cv2.SVM()
        self.svm.load(learned_path + 'svm.xml')

        #A
        prototxt_path   = learned_path + 'groupA/const.prototxt'
        caffemodel_path = learned_path + 'groupA/model.caffemodel.h5'
        mean_path       = learned_path + 'groupA/mean.binaryproto'

        mean_blob = caffe_pb2.BlobProto()
        with open(mean_path)as f:
            mean_blob.ParseFromString(f.read()) 
        mean_array = np.asarray(mean_blob.data, dtype = np.float32).reshape((mean_blob.channels, mean_blob.height, mean_blob.width))

        self.classifierA = caffe.Classifier(prototxt_path, caffemodel_path, mean = mean_array, raw_scale = 255, channel_swap = (2, 1, 0))

        #B
        prototxt_path   = learned_path + 'groupB/const.prototxt'
        caffemodel_path = learned_path + 'groupB/model.caffemodel.h5'
        mean_path       = learned_path + 'groupB/mean.binaryproto'

        with open(mean_path)as f:
            mean_blob.ParseFromString(f.read()) 
        mean_array = np.asarray(mean_blob.data, dtype = np.float32).reshape((mean_blob.channels, mean_blob.height, mean_blob.width))

        self.classifierB = caffe.Classifier(prototxt_path, caffemodel_path, mean = mean_array, raw_scale = 255, channel_swap = (2, 1, 0))

        #C
        prototxt_path   = learned_path + 'groupC/const.prototxt'
        caffemodel_path = learned_path + 'groupC/model.caffemodel.h5'
        mean_path       = learned_path + 'groupC/mean.binaryproto'

        with open(mean_path)as f:
            mean_blob.ParseFromString(f.read()) 
        mean_array = np.asarray(mean_blob.data, dtype = np.float32).reshape((mean_blob.channels, mean_blob.height, mean_blob.width))

        self.classifierC = caffe.Classifier(prototxt_path, caffemodel_path, mean = mean_array, raw_scale = 255, channel_swap = (2, 1, 0))

        #D
        prototxt_path   = learned_path + 'groupD/const.prototxt'
        caffemodel_path = learned_path + 'groupD/model.caffemodel.h5'
        mean_path       = learned_path + 'groupD/mean.binaryproto'

        with open(mean_path)as f:
            mean_blob.ParseFromString(f.read()) 
        mean_array = np.asarray(mean_blob.data, dtype = np.float32).reshape((mean_blob.channels, mean_blob.height, mean_blob.width))

        self.classifierD = caffe.Classifier(prototxt_path, caffemodel_path, mean = mean_array, raw_scale = 255, channel_swap = (2, 1, 0))

    
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

        #get HOG feature
        HOGfeature = np.array(self.hog.compute(image), dtype=np.float32)

        #convert for caffe
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = skimage.img_as_float(image).astype(np.float32)

          #
        if int(self.svm.predict(HOGfeature)) == 0:
            start2 = time.time()        
            pred = np.argmax(self.classifierA.predict([image], oversample = False))
            print 'time1: ' + str(time.time() - start1)
            print 'time2: ' + str(time.time() - start2)
            print names[0][pred]
            raw_input('#####Type enter key to start#####')
            result = ObjRecResult(obj_rec_result = pred + 1)
            self._obj_rec_action_server.set_succeeded(result)

        elif int(self.svm.predict(HOGfeature)) == 1:
            start2 = time.time()        
            pred = np.argmax(self.classifierB.predict([image], oversample = False))
            print 'time1: ' + str(time.time() - start1)
            print 'time2: ' + str(time.time() - start2)
            print names[1][pred]
            raw_input('#####Type enter key to start#####')
            result = ObjRecResult(obj_rec_result = pred + 1)
            self._obj_rec_action_server.set_succeeded(result)

        elif int(self.svm.predict(HOGfeature)) == 2:
            start2 = time.time()        
            pred = np.argmax(self.classifierC.predict([image], oversample = False))
            print 'time1: ' + str(time.time() - start1)
            print 'time2: ' + str(time.time() - start2)
            print names[2][pred]
            raw_input('#####Type enter key to start#####')
            result = ObjRecResult(obj_rec_result = pred + 1)
            self._obj_rec_action_server.set_succeeded(result)

        elif int(self.svm.predict(HOGfeature)) == 3:
            start2 = time.time()        
            pred = 6            
            print 'time1: ' + str(time.time() - start1)
            print 'time2: ' + str(time.time() - start2)
            print '/cockie'
            raw_input('#####Type enter key to start#####')
            result = ObjRecResult(obj_rec_result = pred + 1)
            self._obj_rec_action_server.set_succeeded(result)

        elif int(self.svm.predict(HOGfeature)) == 4:
            start2 = time.time()        
            pred = np.argmax(self.classifierD.predict([image], oversample = False))
            print 'time1: ' + str(time.time() - start1)
            print 'time2: ' + str(time.time() - start2)
            print names[4][pred]
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
