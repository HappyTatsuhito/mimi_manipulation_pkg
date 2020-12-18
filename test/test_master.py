#!/usr/bin/env python
# -*- coding: utf-8 -*

<<<<<<< HEAD
import sys
=======
>>>>>>> eaa596d... テストコード用のtestディレクトリの作成 by Laptop 20/12/18
import rospy
from mimi_manipulation_pkg.srv import ManipulateSrv


if __name__ == "__main__":
<<<<<<< HEAD
    args = sys.argv

    rospy.init_node("test_master")

    manipulation_master = rospy.ServiceProxy("/manipulation", ManipulateSrv)

    rospy.wait_for_service("/manipulation")
<<<<<<< HEAD
    res = manipulation_master(args[-1])
    rospy.loginfo("manipulation_master\n", res.result)
=======
    res = manipulation_master(target_name)
    rospy.loginfo("manipulation_master")
    rospy.loginfo(res.result)
>>>>>>> 0c855a3... enum testのデバッグ1 21/3/10 by Jetson
=======
    rospy.init_node("test_master")
    print "ok"
>>>>>>> eaa596d... テストコード用のtestディレクトリの作成 by Laptop 20/12/18
