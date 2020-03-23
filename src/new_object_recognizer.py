#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import time
import math
import rospy
import rosparam
import actionlib
# -- ros msgs --
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
from mimi_manipulation_pkg.msg import ImageRange
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeCount
# -- action msgs --
from mimi_manipulation_pkg.msg import *


class MimiControl(object):
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)

    def moveBase(self, rad_speed):
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

    ###
    # mimi_common_pkgから指定角度回転するモジュールを持ってくる
    # searchObjectやlocalizeObjectで使用する
    ###

        
class CallDetector(object):
    def __init__(self):
        detector_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.detectorCB)
        self.image_range_pub = rospy.Publisher('/object/image_range',ImageRange,queue_size=1)

        self.object_centroid = Point()
        self.centroid_flg = False
        
    def detectorCB(self, res):
        self.object_centroid = res
        self.centroid_flg = True
        

class RecognizeTools(object):
    def __init__(self):
        bounding_box_sub  = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
        recog_service_server = rospy.Service('/object/recognize',RecognizeCount,self.countObject)

        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を保存
        self.update_flg = False # darknetからpublishされたかどうかの確認

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def initializeBBox(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5 and self.update_flg:
                self.bbox = []
                self.update_flg = False
                rospy.loginfo('initialize') # test
            rate.sleep()

    def searchObject(self, object_name='None'):
        mimi_control = MimiControl()
        if type(object_name) != str:
            object_name = object_name.target
        search_flg = False
        search_count = 0
        while not search_flg and search_count < 10 and not rospy.is_shutdowm():
            #rotate
            if object_name == 'None':
                search_flg = bool(len(self.bbox))
            elif object_name == 'any':
                search_flg = bool(len(list(set(self.object_dict['any']) & set(self.bbox))))
            else:
                search_flg = object_name in self.bbox
        return search_flg
            
    def countObject(self, object_name='None', bb=None):
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target
        object_list = []
        for i in range(len(bb)):
            object_list.append(bb[i].Class)
        object_count = object_list.count(object_name)
        if object_name == 'any':
            any_dict = {}
            for i in range(len(object_list)):
                if object_list[i] in object_dict['any']:
                    any_dict[object_list[i]] = bb[i].xmin
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            object_list = sorted_any_dict.keys()
        return object_count, object_list

    def localizeObject(self, object_name='None', bb=None):
        call_detector = CallDetector()

        
class RecognizeAction(object):
    def __init__(self):
        self.act = actionlib.SimpleActionServer('/manipulation/localize',
                                                ObjectRecognizerAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.search_count = 0
        self.move_count = 0

        self.act.start()

    def actionPreempt(self):
        rospy.loginfo('preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self, goal):
        rospy.loginfo('start action %s'%(goal.recog_goal))
        target_name = goal.recog_goal
        localize_feedback = ObjectRecognizerFeedback()
        localize_result = ObjectRecognizerResult()
        recognize_tools = RecognizeTools()
        target_dict = recognize_tools.object_dict
        loop_flg = True
        while loop_flg and not rospy.is_shutdown():
            bb = recognize_tools.bbox
            if target_name in target_dict.keys():
                for i in range(len(target_dict[target_name])):
                    rospy.loginfo(target_dict[target_name][i])
                    object_count, object_list = recognize_tools.countObject(target_dict[target_name][i], bb)
                    if bool(object_count):
                        target_name = target_dict[target_name][i]
                        break
            else:
                object_count, object_list = recognize_tools.countObject(target_name, bb)
#            rospy.sleep(1.0)
            rospy.loginfo('-- recognize result --')
            rospy.loginfo(object_count)
            rospy.loginfo(object_list)
            rospy.loginfo('----------------------')
            range_flg = False
            

if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    obj_recog = ObjectRecognizer()
    obj_recog.initializeBBox()
    rospy.spin()



class ObjectRecognizer:
    def __init__(self):
        # -- topic subscriber --
        bounding_box_sub  = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
        detector_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.detectorCB)
        # -- topic publisher --
        #service化,モジュール化したい
        self.image_range_pub = rospy.Publisher('/object/image_range',ImageRange,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        # -- service server --
        recog_service_server = rospy.Service('/object/recognize',RecognizeCount,self.countObject)
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

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def detectorCB(self, res):
        self.object_centroid = res
        self.centroid_flg = True

    def countObject(self, object_name='None', bb = None):
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target
        object_list = []
        for i in range(len(bb)):
            object_list.append(bb[i].Class)
        object_count = object_list.count(object_name)
        
        if object_name == 'any':
            any_dict = {}
            for i in range(len(object_list)):
                if object_list[i] in object_dict['any']:
                    any_dict[object_list[i]] = bb[i].xmin
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            object_list = sorted_any_dict.keys()
            
        return object_count, object_list

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
                    object_count, object_list = self.countObject(self.object_dict[target_name][i], bb)
                    if bool(object_count):
                        target_name = self.object_dict[target_name][i]
                        break
            else:
                object_count, object_list = self.countObject(target_name, bb)
#            rospy.sleep(1.0)
            rospy.loginfo('-- recognize result --')
            rospy.loginfo(object_count)
            rospy.loginfo(object_list)
            rospy.loginfo('----------------------')
            range_flg = False
            # ここらへんをもう少し綺麗に書きたい
            if target_name == 'None' and not bool(object_count):# 適当に見えたものを掴むための処理（ここいらんかも）
                list_num = 0
                range_flg = True
            elif bool(object_count):# 指定のものを掴むための処理
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
