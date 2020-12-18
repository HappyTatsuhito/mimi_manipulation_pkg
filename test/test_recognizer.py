#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import rospy
from mimi_manipulation_pkg.srv import RecognizeCount, RecognizeFind, RecognizeLocalize


if __name__ == "__main__":
    args = sys.argv
    target_name = args[-1]

    rospy.init_node("test_recognizer")

    count_object = rospy.ServiceProxy("/recog/count", RecognizeCount)
    find_object = rospy.ServiceProxy("/recog/find", RecognizeFind)
    localize_object = rospy.ServiceProxy("/recog/localize", RecognizeLocalize)

    rospy.wait_for_service("/recog/count")
    res = count_object(target_name)
    rospy.loginfo("module : count\n", res.object_num, res.object_list)
    
    rospy.wait_for_service("/recog/find")
    res = find_object(target_name)
    rospy.loginfo("\nmodule : find\n", res.result)

    rospy.wait_for_service("/recog/localize")
    res = localize_object(target_name)
    rospy.loginfo("\nmodule : localize\n", res.centroid_point)
