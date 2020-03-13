#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import sys
import time
import math
import rosparam
import actionlib
# -- ros msgs --
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
from object_recognizer.msg import ImageRange
# -- ros srvs --
from object_recognizer.srv import RecognizeExistence
# -- action msgs --
from manipulation.msg import *

class ObjectRecognizer:
    def __init__(self):
        # -- topic subscriber --
        bounding_box_sub  = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.BoundingBoxCB)
        detector_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.detectorCB)
        # -- topic publisher --
        #service化,モジュール化したい
        self.image_range_pub = rospy.Publisher('/object/image_range',ImageRange,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        # -- service server --
        recog_service_server = rospy.Service('/object/recognize',RecognizeExistence,self.recognizeObject)
        # -- action server --
        self.act = actionlib.SimpleActionServer('/manipulation/localize',
                                                ObjectRecognizerAction,
                                                execute_cb = self.localizeObject,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)
        # -- instance variables --
        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = 'None'
        self.update_time = 0 # darknetからpublishされた時刻を保存
        self.update_flg = False # darknetからpublishされたかどうかの確認
        self.object_centroid = Point()
        self.centroid_flg = False
        self.preempt_flg = False
        self.search_count = 0
        self.move_count = 0

        self.act.start()

    def BoundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def detectorCB(self, res):
        self.object_centroid = res
        self.centroid_flg = True

    def recognizeObject(self, object_name='None', bb = None):
        if bb is None:
            bb = self.bbox
        #rospy.loginfo('recognize')
        if type(object_name) != str:
            object_name = object_name.target
        object_list = []
        if bb == 'None':
            return False, []
        for i in range(len(bb)):
            object_list.append(bb[i].Class)
        object_existence = object_name in object_list
        return object_existence, object_list

    def actionPreempt(self):
        rospy.loginfo('preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True
        
    def localizeObject(self, goal):
        rospy.loginfo('start action %s'%(goal.recog_goal))
        target_name = goal.recog_goal
        localize_feedback = ObjectRecognizerFeedback()
        localize_result = ObjectRecognizerResult()
        loop_flg = True
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        while loop_flg and not rospy.is_shutdown():
            bb = self.bbox
            if target_name in self.object_dict.keys():
                for i in range(len(self.object_dict[target_name])):
                    rospy.loginfo(self.object_dict[target_name][i])
                    object_existence, object_list = self.recognizeObject(self.object_dict[target_name][i], bb)
                    if object_existence == True:
                        target_name = self.object_dict[target_name][i]
                        break
            else:
                object_existence, object_list = self.recognizeObject(target_name, bb)
#            rospy.sleep(1.0)
            rospy.loginfo('-- recognize result --')
            rospy.loginfo(object_existence)
            rospy.loginfo(object_list)
            rospy.loginfo('----------------------')
            range_flg = False
            # ここらへんをもう少し綺麗に書きたい
            if target_name == 'None' and not object_existence:# 適当に見えたものを掴むための処理（ここいらんかも）
                list_num = 0
                range_flg = True
            elif object_existence:# 指定のものを掴むための処理
                list_num = object_list.index(target_name)
                range_flg = True
            if range_flg:
                object_image_range = ImageRange()
                object_image_range.top = bb[list_num].ymin
                object_image_range.bottom = bb[list_num].ymax
                object_image_range.left = bb[list_num].xmin
                object_image_range.right = bb[list_num].xmax
                rospy.sleep(0.2)
                self.image_range_pub.publish(object_image_range)
                while self.centroid_flg == False and not rospy.is_shutdown():
                    pass
                object_coordinate = self.object_centroid
                self.centroid_flg = False
                #print object_coordinate
                if not math.isnan(object_coordinate.x):# 物体が正面になるように回転する処理
                    object_coordinate.y += 0.08 # calibrate RealSenseCamera d435
                    object_angle = math.atan2(object_coordinate.y, object_coordinate.x)
                    if abs(object_angle) > 0.07:
                        rospy.loginfo('There is not object in front.')
                        cmd.angular.z = object_angle * 3.5 #要調整
                        if abs(cmd.angular.z) < 0.7:
                            cmd.angular.z = int(cmd.angular.z/abs(cmd.angular.z))*0.7
                        rospy.loginfo('cmd.angura.z : %s'%(object_angle))
                        self.cmd_vel_pub.publish(cmd)
                        cmd.angular.z = 0
                        rospy.sleep(4.0)
                        # retry
                    else:
                        # success
                        loop_flg = False
                else:
                    #前後進
                    self.move_count += 1
                    range_flg = False
                    move_range = -0.4*(((self.move_count)%4)/2)+0.2
                    self.moveBase(move_range)
                    rospy.sleep(3.0)
            else:
                #回転
                self.search_count += 1
                cmd.angular.z = -2.0*(((self.search_count)%4)/2)+1.0
                self.cmd_vel_pub.publish(cmd)
                cmd.angular.z = 0
                rospy.sleep(4.0)
            if loop_flg:
                localize_feedback.recog_feedback = range_flg
                self.act.publish_feedback(localize_feedback)
            range_flg = False
            if self.preempt_flg:
                self.preempt_flg = False
                break
        else:
            self.search_count = 0
            self.move_count = 0
            rospy.loginfo('Succeeded')
            localize_result.recog_result = self.object_centroid
            self.act.set_succeeded(localize_result)

    def moveBase(self,rad_speed):
        cmd = Twist()
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*speed_i
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*(10-speed_i)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_vel_pub.publish(cmd)

    def initializeObject(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5 and self.update_flg:
                self.bbox = 'None'
                self.update_flg = False
                rospy.loginfo('initialize') # test
            rate.sleep()
            
            
if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    obj_recog = ObjectRecognizer()
    obj_recog.initializeObject()
    rospy.spin()
