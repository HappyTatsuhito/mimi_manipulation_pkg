#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from mimi_manipulation_pkg.srv import ManipulateSrv


if __name__ == "__main__":
    rospy.init_node("test_master")
    print "ok"
