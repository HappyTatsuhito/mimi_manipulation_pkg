#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeFind, RecognizeCount, RecognizeLocalize

if __name__ == '__main__':
    args = sys.argv
    target_name = args[-1]
    
    rospy.init_node('test_recognizer')

    find_object = rospy.ServiceProxy("/recognize/find", RecognizeFind)
    count_object = rospy.ServiceProxy("/recognize/count", RecognizeCount)
    localize_object = rospy.ServiceProxy("/recognize/localize", RecognizeLocalize)

    rospy.wait_for_service("/recognize/find")
    res = find_object(target_name)
    rospy.loginfo("Find Ojbect")
    rospy.loginfo(res.result)

    rospy.wait_for_service("/recognize/count")
    res = count_object(target_name)
    rospy.loginfo("Count Ojbect")
    rospy.loginfo(res)

    rospy.wait_for_service("/recognize/localize")
    res = localize_object(target_name)
    rospy.loginfo("Localize Ojbect")
    rospy.loginfo(res)
