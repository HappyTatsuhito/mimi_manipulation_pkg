#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
from mimi_manipulation_pkg.srv import RecognizeCount, RecognizeFind, RecognizeLocalize


if __name__ == "__main__":
    args = sys.argv
    target_name = args[-1]

    rospy.init_node("test_recognizer")

    count_object = rospy.ServiceProxy("/recognize/count", RecognizeCount)
    find_object = rospy.ServiceProxy("/recognize/find", RecognizeFind)
    localize_object = rospy.ServiceProxy("/recognize/localize", RecognizeLocalize)

    rospy.sleep(1.0)
    rospy.loginfo("wait")

    rospy.wait_for_service("/recog/count")
    rospy.sleep(1.0)
    rospy.loginfo("wait2")
    res = count_object(target_name)
    rospy.loginfo("module : count\n", res.object_num, res.object_list)
    
    rospy.wait_for_service("/recog/find")
    res = find_object(target_name)
    rospy.loginfo("\nmodule : find\n", res.result)

    rospy.wait_for_service("/recog/localize")
    res = localize_object(target_name)
    rospy.loginfo("\nmodule : localize\n", res.centroid_point)
