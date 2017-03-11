#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#mapとbase_linkの座標変換を生成するROSノード
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
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])

    tf_listener = tf.TransformListener()

    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                (translation, rotation) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except:
                continue
            break

        euler = euler_from_quaternion([rotation[0], rotation[1], rotation[2], rotation[3]])

        print 'x: ' + str(round(translation[0], 3)) + ' y: ' + str(round(translation[1], 3)) + ' yaw: ' + str(round(euler[2], 3))

        map2base_link = tf.TransformBroadcaster()
        map2base_link.sendTransform((translation[0], translation[1], translation[2]), (rotation[0], rotation[1], rotation[2], rotation[3]), rospy.Time.now(), "base_link", "map")
    
        main_rate.sleep()
