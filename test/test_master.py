#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
# -- ros srvs --
from mimi_manipulation_pkg.srv import ManipulateSrv

if __name__ == '__main__':
    args = sys.argv
    target_name = args[-1]
    
    rospy.init_node('test_master')

    manipulation_master = rospy.ServiceProxy('/manipulation', ManipulateSrv)

    rospy.wait_for_service('/manipulation')
    res = manipulation_master(target_name)
    rospy.loginfo('manipulation_master')
    rospy.loginfo(res.result)
