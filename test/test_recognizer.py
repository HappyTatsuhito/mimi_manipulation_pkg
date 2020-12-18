#!/usr/bin/env python
# -*- coding: utf-8 -*
<<<<<<< HEAD
<<<<<<< HEAD

import sys
import rospy
<<<<<<< HEAD
from mimi_manipulation_pkg.srv import RecognizeCount, RecognizeFind, RecognizeLocalize
=======
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeFind, RecognizeCount, RecognizeLocalize
>>>>>>> 16e7e2b... testのデバッグ完了 21/3/10 by Jetson
=======

import sys
import rospy
from mimi_manipulation_pkg.srv import RecognizeCount, RecognizeFind, RecognizeLocalize
>>>>>>> e6bb873... create testcode by Laptop 20/12/18


if __name__ == "__main__":
    args = sys.argv
    target_name = args[-1]

    rospy.init_node("test_recognizer")

<<<<<<< HEAD
<<<<<<< HEAD
    count_object = rospy.ServiceProxy("/recognize/count", RecognizeCount)
<<<<<<< HEAD
    find_object = rospy.ServiceProxy("/recognize/find", RecognizeFind)
    localize_object = rospy.ServiceProxy("/recognize/localize", RecognizeLocalize)

    rospy.sleep(1.0)
    rospy.loginfo("wait")

    rospy.wait_for_service("/recog/count")
    rospy.sleep(1.0)
    rospy.loginfo("wait2")
=======
    count_object = rospy.ServiceProxy("/recog/count", RecognizeCount)
    find_object = rospy.ServiceProxy("/recog/find", RecognizeFind)
    localize_object = rospy.ServiceProxy("/recog/localize", RecognizeLocalize)
=======
    count_object = rospy.ServiceProxy("/recognize/count", RecognizeCount)
    find_object = rospy.ServiceProxy("/recognize/find", RecognizeFind)
    localize_object = rospy.ServiceProxy("/recognize/localize", RecognizeLocalize)
>>>>>>> 3a3c737... renamed recog -> recognize by Laptop 20/12/18

    rospy.wait_for_service("/recog/count")
>>>>>>> e6bb873... create testcode by Laptop 20/12/18
    res = count_object(target_name)
    rospy.loginfo("module : count\n", res.object_num, res.object_list)
    
    rospy.wait_for_service("/recog/find")
    res = find_object(target_name)
    rospy.loginfo("\nmodule : find\n", res.result)

    rospy.wait_for_service("/recog/localize")
    res = localize_object(target_name)
    rospy.loginfo("\nmodule : localize\n", res.centroid_point)
<<<<<<< HEAD
=======
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
>>>>>>> 16e7e2b... testのデバッグ完了 21/3/10 by Jetson
=======
>>>>>>> eaa596d... テストコード用のtestディレクトリの作成 by Laptop 20/12/18
=======
>>>>>>> e6bb873... create testcode by Laptop 20/12/18
