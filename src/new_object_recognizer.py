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
#from darknet_ros_msgs.msg import BoundingBoxes
from mimi_manipulation_pkg.msg import ImageRange
# -- ros srvs --
from mimi_manipulation_pkg.srv import RecognizeCount
# -- action msgs --
from mimi_manipulation_pkg.msg import *

sys.path.insert(0, '/home/demlab/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import BaseCarrier


class MimiControl(BaseCarrier):
    def __init__(self):
        super(MimiControl,self).__init__()

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
        recog_search_srv = rospy.Service('/recog/search',RecognizeSearch,self.searchObject)
        recog_count_srv = rospy.Service('/recog/count',RecognizeCount,self.countObject)
        recog_localize_srv = rospy.Service('/recog/localize',RecognizeLocalize,self.localizeObject)

        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を記録
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
            search_count += 1
            rotation_angle = 45 - (((search_count)%4)/2) * 90
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(2.0)
            if object_name == 'None':
                search_flg = bool(len(self.bbox))
            elif object_name == 'any':
                search_flg = bool(len(list(set(self.object_dict['any'])&set(self.bbox))))
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

    def localizeObject(self, object_name, bb=None):
        Detector = CallDetector()
        if bb is None:
            bb = self.bbox
        image_range = ImageRange()
        image_range.top = bb[object_name].ymin
        image_range.bottom = bb[object_name].ymax
        image_range.left = bb[object_name].xmin
        image_range.right = bb[object_name].xmax
        rospy.sleep(0.2)
        Detector.image_range_pub.publish(image_range)
        while Detector.centroid_flg == False and not rospy.is_shutdown():
            pass
        object_centroid = Detector.object_centroid
        Detector.centroid_flg = False
        return object_centroid
        
        
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
        target_name = goal.recog_goal
        rospy.loginfo('start action >> recognize %s'%(target_name))
        action_feedback = ObjectRecognizerFeedback()
        action_result = ObjectRecognizerResult()
        target_dict = recognize_tools.object_dict
        recognize_tools = RecognizeTools()
        mimi_control = MimiControl()
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
            ###
            #if bool(object_count):
            #    list_num = object_list.index(target_name)
            #    range_flg = True
            range_flg = bool(object_count)
            ###
            if range_flg:
                object_centroid = recognize_tools.localizeObject(target_name)
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_centroid.y += 0.08 # calibrate RealSenseCamera d435
                    object_angle = math.atan2(object_centroide.y, object_centroid.x)/math.pi*180
                    if abs(object_angle) > 4.5:
                        # retry
                        rospy.loginfo('There is not object in front.')
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                    else:
                        # success
                        loop_flg = False
                else:
                    #前後進
                    pass
            else:
                search_flg = recognize_tools.searchObject()
            if loop_flg:
                action_feedback.recog_feedback = range_flg
                self.act.publish_feedback(action_feedback)
            range_flg = False
            if self.preempt_flg:
                self.preempt_flg = False
                break
        else:
            rospy.loginfo('Succeeded')
            action_result.recog_result = object_centroid
            self.act.set_succeeded(action_result)
                
                
if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    obj_recog = ObjectRecognizer()
    obj_recog.initializeBBox()
    rospy.spin()
