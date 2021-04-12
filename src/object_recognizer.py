#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import time
import math
import numpy
import rospy
import rosparam
import actionlib
# -- ros msgs --
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeFind, RecognizeCount, RecognizeLocalize, DetectDepth
# -- action msgs --
from mimi_manipulation_pkg.msg import *

class MimiControl(object):
    def __init__(self):
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)

        self.twist_value = Twist()

    def angleRotation(self, degree):
        while degree > 180:
            degree = degree - 360
        while degree < -180:
            degree = degree + 360
        angular_speed = 50.0 #[deg/s]
        target_time = abs(1.76899*(degree /angular_speed))  #[s]
        if degree >= 0:
            self.twist_value.angular.z = (angular_speed * 3.14159263 / 180.0) #rad
        elif degree < 0:
            self.twist_value.angular.z = -(angular_speed * 3.14159263 / 180.0) #rad
        rate = rospy.Rate(500)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.cmd_vel_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_value)

    def moveBase(self, rad_speed):
        for speed_i in range(10):
            self.twist_value.linear.x = rad_speed*0.05*speed_i
            self.twist_value.angular.z = 0
            self.cmd_vel_pub.publish(self.twist_value)
            rospy.sleep(0.1)
        for speed_i in range(10):
            self.twist_value.linear.x = rad_speed*0.05*(10-speed_i)
            self.twist_value.angular.z = 0
            self.cmd_vel_pub.publish(self.twist_value)
            rospy.sleep(0.1)
        self.twist_value.linear.x = 0
        self.twist_value.angular.z = 0
        self.cmd_vel_pub.publish(self.twist_value)

        
class CallDetector(object):
    def __init__(self):
        self.detect_depth = rospy.ServiceProxy('/detect/depth',DetectDepth)

        self.object_centroid = Point()
        
    def detectorService(self, center_x, center_y):
        rospy.wait_for_service('/detect/depth')
        res = self.detect_depth(center_x, center_y)
        self.object_centroid = res.centroid_point
        rospy.loginfo(self.object_centroid)
        

class RecognizeTools(object):
    def __init__(self):
        bounding_box_sub  = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
        recognize_find_srv = rospy.Service('/recognize/find',RecognizeFind,self.findObject)
        recognize_count_srv = rospy.Service('/recognize/count',RecognizeCount,self.countObject)
        recognize_localize_srv = rospy.Service('/recognize/localize',RecognizeLocalize,self.localizeObject)

        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def createBboxList(self,bb):
        bbox_list = []
        for i in range(len(bb)):
            bbox_list.append(bb[i].Class)
        return bbox_list

    def initializeBBox(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5 and self.update_flg:
                self.bbox = []
                self.update_flg = False
                rospy.loginfo('initialize')
            rate.sleep()

    def findObject(self, object_name='None'):
        rospy.loginfo('module type : Find')
        mimi_control = MimiControl()
        
        if type(object_name) != str:
            object_name = object_name.target_name
            
        find_flg, _ = self.countObject(object_name)
        loop_count = 0
        
        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1
            
            rotation_angle = 45 - (((loop_count)%4)/2) * 90
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)

            bbox_list = self.createBboxList(self.bbox)
            if object_name == 'None':
                find_flg = bool(len(bbox_list))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(bbox_list))))
            else:
                find_flg = object_name in bbox_list
        return find_flg
            
    def countObject(self, object_name='None', bb=None):
        rospy.loginfo('module type : Count')
        
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name
            
        object_list = []
        bbox_list = self.createBboxList(bb)
        
        if object_name == 'any':
            any_dict = {}
            for i in range(len(bbox_list)):
                if bbox_list[i] in self.object_dict['any']:
                    any_dict[bbox_list[i]] = bb[i].xmin
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            for i in range(len(sorted_any_dict)):
                object_list.append(sorted_any_dict[i][0])
            object_count = len(sorted_any_dict)
        else:
            object_count = bbox_list.count(object_name)
            object_list = bbox_list
        return object_count, object_list

    def localizeObject(self, object_name='None', bb=None):
        rospy.loginfo('module type : Localize')
        Detector = CallDetector()
        
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name
            
        object_centroid = Point()
        object_centroid.x = numpy.nan
        object_centroid.y = numpy.nan
        object_centroid.z = numpy.nan

        bbox_list = self.createBboxList(bb)
        object_count, object_list = self.countObject(object_name)

        if object_name == 'any':
            exist_flg = bool(len(object_list))
            if exist_flg: object_name = object_list[0]
        else:
            exist_flg = bool(object_count)

        if not exist_flg:
            return object_centroid
        
        index_num = bbox_list.index(object_name)
        center_x = int((bb[index_num].ymin + bb[index_num].ymax)/2)
        center_y = int((bb[index_num].xmin + bb[index_num].xmax)/2)
        rospy.sleep(0.5)
        Detector.detectorService(center_x, center_y)
        object_centroid = Detector.object_centroid
        return object_centroid
        
        
class RecognizeAction(object):
    def __init__(self):
        self.act = actionlib.SimpleActionServer('/manipulation/localize',
                                                ObjectRecognizerAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.preempt_flg = False
        self.act.start()

        self.recognize_tools = RecognizeTools()
        self.recognize_tools.initializeBBox()

    def actionPreempt(self):
        rospy.loginfo('preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self, goal):
        target_name = goal.recognize_goal
        rospy.loginfo('start action >> recognize %s'%(target_name))
        action_feedback = ObjectRecognizerFeedback()
        action_result = ObjectRecognizerResult()
        mimi_control = MimiControl()
        #recognize_tools = RecognizeTools()
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognize_tools.bbox
            object_count, _ = self.recognize_tools.countObject(target_name, bb)
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = self.recognize_tools.localizeObject(target_name, bb)
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
                    if abs(object_angle) < 4.5:
                        # success
                        rospy.loginfo('Succeeded')
                        action_result.recognize_result = object_centroid
                        self.act.set_succeeded(action_result)
                        break
                    else:
                        # retry
                        rospy.loginfo('There is not object in front.')
                        object_angle *= 1.5 # kobukiが重いので
                        if abs(object_angle) < 10: object_angle=object_angle/abs(object_angle)*10
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                else:
                    #前後進
                    move_count += 1
                    move_range = -0.8*(((move_count)%4)/2)+0.4
                    mimi_control.moveBase(move_range)
                    exist_flg = False
            else:
                find_flg = self.recognize_tools.findObject(target_name)
                exist_flg = find_flg
            action_feedback.recognize_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
            rospy.sleep(1.0) #preemptのズレ調整用
            if self.preempt_flg:
                self.preempt_flg = False
                break


if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognize_action = RecognizeAction()
    #recognize_action.recognize_tools.initializeBBox()
    rospy.spin()
