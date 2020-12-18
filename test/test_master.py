#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
from mimi_manipulation_pkg.srv import ManipulateSrv


if __name__ == "__main__":
    args = sys.argv

    rospy.init_node("test_master")

    manipulation_master = rospy.ServiceProxy("/manipulation", ManipulateSrv)

    rospy.wait_for_service("/manipulation")
    res = manipulation_master(args[-1])
    rospy.loginfo("manipulation_master\n", res.result)
