#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
from std_msgs.msg import Float64
# -- ros srvs --
from mimi_manipulation_pkg.srv import ManipulateSrv

if __name__ == '__main__':
    args = sys.argv
    target_name = args[-1]

    rospy.init_node('test_grasper')

    head_controller = rospy.Publisher('/servo/head', Float64, queue_size=1)
    arm_changer = rospy.ServiceProxy('/servo/arm', ManipulateSrv)

    rospy.sleep(1.0)
    
    rospy.loginfo('Controll Head')
    head_controller.publish(0.0)

    rospy.wait_for_service('/servo/arm')
    _ = arm_changer('carry')
    rospy.loginfo('Change Arm')
