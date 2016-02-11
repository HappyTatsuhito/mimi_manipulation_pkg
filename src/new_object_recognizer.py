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
from mimi_manipulation_pkg.srv import RecognizeFind, RecognizeCount, RecognizeLocalize
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
        angular_speed = 90.0 #[deg/s]
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
        recog_find_srv = rospy.Service('/recog/find',RecognizeFind,self.findObject)
        recog_count_srv = rospy.Service('/recog/count',RecognizeCount,self.countObject)
        recog_localize_srv = rospy.Service('/recog/localize',RecognizeLocalize,self.localizeObject)

        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

        rospy.loginfo("start tools")

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def initializeBBox(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5:# and not self.update_flg:
                self.bbox = []
                self.update_flg = False
#                rospy.loginfo('initialize') # test
            rate.sleep()

    def findObject(self, object_name='None'):
        mimi_control = MimiControl()
        if type(object_name) != str:
            object_name = object_name.target
<<<<<<< HEAD
        find_flg = False
        find_count = 0
<<<<<<< HEAD
        while not find_flg and find_count < 10 and not rospy.is_shutdowm():
=======
<<<<<<< HEAD
        while not find_flg and find_count < 10 and not rospy.is_shutdown():
            print type(self.bbox)
=======
        while not find_flg and find_count <= 3 and not rospy.is_shutdown():
>>>>>>> 0268ad5... first commit by Laptop 10/8
            bbox_list = self.createBboxList(self.bbox)
<<<<<<< HEAD
>>>>>>> 0d8b65f... first commit by Laptop 10/8
            #rotate
=======
>>>>>>> 1ea6bd2... changed the content of the action　by Laptop 10/12
            find_count += 1
            rotation_angle = 45 - (((find_count)%4)/2) * 90
=======
        find_flg, _ = self.countObject(object_name)
        loop_count = 0
        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1
            rotation_angle = 45 - (((loop_count)%4)/2) * 90
>>>>>>> ae4f529... debuged the action by Jetson 20/10/31
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)
            bbox_list = self.createBboxList(self.bbox)
            if object_name == 'None':
                find_flg = bool(len(self.bbox))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(self.bbox))))
            else:
                find_flg = object_name in self.bbox
        return find_flg
            
    def countObject(self, object_name='None', bb=None):
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target
        object_list = []
        bbox_list = self.createBboxList(bb)
        if object_name == 'any':
            any_dict = {}
            for i in range(len(object_list)):
                if object_list[i] in object_dict['any']:
                    any_dict[object_list[i]] = bb[i].xmin
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            for i in range(len(sorted_any_dict)):
                object_list.append(sorted_any_dict[i][0])
            object_count = len(sorted_any_dict)
        else:
            object_count = bbox_list.count(object_name)
            object_list = bbox_list
        return object_count, object_list

    def localizeObject(self, object_name, bb=None):
        Detector = CallDetector()
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target
        object_centroid = Point()
        bbox_list = self.createBboxList(bb)
        object_count, _ = self.countObject(object_name)
        exist_flg = bool(object_count)
        if not exist_flg:
            object_centroid.x = numpy.nan
            object_centroid.y = numpy.nan
            object_centroid.z = numpy.nan
            return object_centroid
        index_num = bbox_list.index(object_name)
        image_range = ImageRange()
        image_range.top = bb[index_num].ymin
        image_range.bottom = bb[index_num].ymax
        image_range.left = bb[index_num].xmin
        image_range.right = bb[index_num].xmax
        rospy.sleep(0.5)
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

        self.preempt_flg = False
        self.recognize_tools = RecognizeTools()
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
        mimi_control = MimiControl()
        #recognize_tools = RecognizeTools()
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognize_tools.bbox
            object_count, _ = self.recognize_tools.countObject(target_name, bb)
            rospy.loginfo(object_count)
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = self.recognize_tools.localizeObject(target_name, bb)
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_centroid.y += 0.08 # calibrate RealSenseCamera d435
                    object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
                    if abs(object_angle) < 4.5:
                        # success
                        rospy.loginfo('Succeeded')
                        action_result.recog_result = object_centroid
                        self.act.set_succeeded(action_result)
                        break
                    else:
                        # retry
                        rospy.loginfo('There is not object in front.')
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                else:
                    #前後進
                    move_count += 1
                    move_range = -0.4*(((move_count)%4)/2)+0.2
                    exist_flg = False
            else:
                find_flg = self.recognize_tools.findObject(target_name)
                exist_flg = find_flg
            action_feedback.recog_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
            if self.preempt_flg:
                self.preempt_flg = False
                break


if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognize_action = RecognizeAction()
#    recognize_tools = RecognizeTools()
#    recognize_tools.initializeBBox()
    rospy.spin()
