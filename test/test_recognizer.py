#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeCount, RecognizeFind, RecognizeLocalize

if __name__ == '__main__':
    args = sys.argv
    target_name = args[-1]
    
    rospy.init_node('test_recognizer')

    find_object = rospy.ServiceProxy("/recognize/find", RecognizeFind)
    count_object = rospy.ServiceProxy("/recognize/count", RecognizeCount)
    localize_ojbect = rospy.ServiceProxy("/recognize/localize", RecognizeLocalize)

    rospy.wait_for_service("/recognize/find")
    res = find_ojbect(target_name)
    rospy.loginfo("module : find\n", res.result)

    rospy.wait_for_service("/recognize/count")
    res = count_ojbect(target_name)
    rospy.loginfo("\nmodule : count\n", res.result)

    rospy.wait_for_service("/recognize/localize")
    res = localize_ojbect(target_name)
    rospy.loginfo("\nmodule : localize\n", res.result)
