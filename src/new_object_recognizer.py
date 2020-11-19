#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys
import time
import math
<<<<<<< HEAD
=======
import numpy
>>>>>>> e485a97... first commit by Laptop 10/8
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
<<<<<<< HEAD

        self.twist_value = Twist()

=======

        self.twist_value = Twist()

>>>>>>> a849b3f... first commit by Laptop 10/8
    def angleRotation(self, degree):
        while degree > 180:
            degree = degree - 360
        while degree < -180:
            degree = degree + 360
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        angular_speed = 70.0 #[deg/s]
=======
        angular_speed = 90.0 #[deg/s]
>>>>>>> e485a97... first commit by Laptop 10/8
=======
        angular_speed = 70.0 #[deg/s]
>>>>>>> 8c7d05c... Recognizerの角度関係のデバッグとmasterとgrasperのパラメータ調整 by Jetson 20/12/5
=======
        angular_speed = 90.0 #[deg/s]
>>>>>>> a849b3f... first commit by Laptop 10/8
        target_time = abs(1.76899*(degree /angular_speed))  #[s]
        if degree >= 0:
            self.twist_value.angular.z = (angular_speed * 3.14159263 / 180.0) #rad
        elif degree < 0:
            self.twist_value.angular.z = -(angular_speed * 3.14159263 / 180.0) #rad
        rate = rospy.Rate(500)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            self.cmd_vel_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_value)
=======
=======
>>>>>>> a849b3f... first commit by Laptop 10/8
            self.pub_twist.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.pub_twist.publish(self.twist_value)
<<<<<<< HEAD
>>>>>>> e485a97... first commit by Laptop 10/8
=======
            self.cmd_vel_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_value)
>>>>>>> 60f5393... Debug for count, find by Jetson 10/8
=======
>>>>>>> a849b3f... first commit by Laptop 10/8
=======
            self.cmd_vel_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_value)
>>>>>>> e3f5aca... Debug for count, find by Jetson 10/8

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                rospy.loginfo('initialize')
=======
#                rospy.loginfo('initialize') # test
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
=======
                rospy.loginfo('initialize') # test
>>>>>>> 2b96295... Fixed third debug by Laptop 11/11
=======
                rospy.loginfo('initialize')
>>>>>>> f412440... Fourth debug by Jetson 11/12
            rate.sleep()

    def findObject(self, object_name='None'):
        mimi_control = MimiControl()
        if type(object_name) != str:
<<<<<<< HEAD
            object_name = object_name.target
<<<<<<< HEAD
<<<<<<< HEAD
        find_flg = False
        find_count = 0
<<<<<<< HEAD
<<<<<<< HEAD
        while not find_flg and find_count < 10 and not rospy.is_shutdowm():
=======
=======
>>>>>>> f9a228d... first commit by Laptop 10/8
<<<<<<< HEAD
        while not find_flg and find_count < 10 and not rospy.is_shutdown():
            print type(self.bbox)
=======
        while not find_flg and find_count <= 3 and not rospy.is_shutdown():
>>>>>>> 0268ad5... first commit by Laptop 10/8
=======
        while not find_flg and find_count <= 3 and not rospy.is_shutdown():
>>>>>>> a849b3f... first commit by Laptop 10/8
            bbox_list = self.createBboxList(self.bbox)
<<<<<<< HEAD
>>>>>>> 0d8b65f... first commit by Laptop 10/8
            #rotate
=======
>>>>>>> 1ea6bd2... changed the content of the action　by Laptop 10/12
            find_count += 1
            rotation_angle = 45 - (((find_count)%4)/2) * 90
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
            object_name = object_name.target_name
>>>>>>> 09036e0... srv変数名の変更＋回転の最低値処理をif文に by Laptop 20/12/18
=======
>>>>>>> 990dd81... debuged the action by Jetson 20/10/31
        find_flg, _ = self.countObject(object_name)
        loop_count = 0
        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1
            rotation_angle = 45 - (((loop_count)%4)/2) * 90
<<<<<<< HEAD
>>>>>>> ae4f529... debuged the action by Jetson 20/10/31
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)
            bbox_list = self.createBboxList(self.bbox)
=======
                rospy.loginfo('initialize') # test
=======
#                rospy.loginfo('initialize') # test
>>>>>>> 4c59708... debuged the action by Jetson 20/10/31
=======
                rospy.loginfo('initialize') # test
>>>>>>> 41ea56a... Fixed third debug by Laptop 11/11
=======
                rospy.loginfo('initialize')
>>>>>>> 4e9f7cb... Fourth debug by Jetson 11/12
            rate.sleep()

    def findObject(self, object_name='None'):
        rospy.loginfo("module type : Find")
        mimi_control = MimiControl()
        if type(object_name) != str:
            object_name = object_name.target_name
        find_flg, _ = self.countObject(object_name)
        loop_count = 0
        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1
            rotation_angle = 45 - (((loop_count)%4)/2) * 90
=======
            print rotation_angle
>>>>>>> e3f5aca... Debug for count, find by Jetson 10/8
            mimi_control.angleRotation(rotation_angle)
<<<<<<< HEAD
<<<<<<< HEAD
            rospy.sleep(2.0)
>>>>>>> e485a97... first commit by Laptop 10/8
=======
            rospy.sleep(4.0)
>>>>>>> 81dc8a8... changed the content of the action　by Laptop 10/12
=======
            rospy.sleep(3.0)
            bbox_list = self.createBboxList(self.bbox)
>>>>>>> df0c7f1... debuged the action by Jetson 20/10/31
=======
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(4.0)
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
=======
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)
            bbox_list = self.createBboxList(self.bbox)
>>>>>>> 990dd81... debuged the action by Jetson 20/10/31
            if object_name == 'None':
                find_flg = bool(len(self.bbox))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(self.bbox))))
            else:
                find_flg = object_name in self.bbox
        return find_flg
            
    def countObject(self, object_name='None', bb=None):
<<<<<<< HEAD
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name
=======
        rospy.loginfo("module type : Count")
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
<<<<<<< HEAD
            object_name = object_name.target
>>>>>>> e485a97... first commit by Laptop 10/8
=======
            object_name = object_name.target_name
>>>>>>> 8f5f794... srv変数名の変更＋回転の最低値処理をif文に by Laptop 20/12/18
        object_list = []
        bbox_list = self.createBboxList(bb)
        if object_name == 'any':
            any_dict = {}
<<<<<<< HEAD
            for i in range(len(object_list)):
                if object_list[i] in object_dict['any']:
                    any_dict[object_list[i]] = bb[i].xmin
=======
            for i in range(len(bbox_list)):
                if bbox_list[i] in self.object_dict['any']:
                    any_dict[bbox_list[i]] = bb[i].xmin
>>>>>>> e485a97... first commit by Laptop 10/8
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            for i in range(len(sorted_any_dict)):
                object_list.append(sorted_any_dict[i][0])
            object_count = len(sorted_any_dict)
        else:
            object_count = bbox_list.count(object_name)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            object_list = bbox_list
        return object_count, object_list

    def localizeObject(self, object_name, bb=None):
=======
            obejct_list = bbox_list
=======
            object_list = bbox_list
>>>>>>> 60f5393... Debug for count, find by Jetson 10/8
=======
            obejct_list = bbox_list
>>>>>>> a849b3f... first commit by Laptop 10/8
=======
            object_list = bbox_list
>>>>>>> e3f5aca... Debug for count, find by Jetson 10/8
        return object_count, object_list

    def localizeObject(self, object_name, bb=None):
        rospy.loginfo("module type : Localize")
>>>>>>> e485a97... first commit by Laptop 10/8
        Detector = CallDetector()
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
<<<<<<< HEAD
<<<<<<< HEAD
            object_name = object_name.target_name
=======
            object_name = object_name.target
<<<<<<< HEAD
>>>>>>> e485a97... first commit by Laptop 10/8
=======
            object_name = object_name.target_name
>>>>>>> 8f5f794... srv変数名の変更＋回転の最低値処理をif文に by Laptop 20/12/18
=======
>>>>>>> a849b3f... first commit by Laptop 10/8
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        rospy.sleep(0.5)
=======
        rospy.sleep(0.2)
>>>>>>> e485a97... first commit by Laptop 10/8
=======
        rospy.sleep(0.5)
>>>>>>> 4c59708... debuged the action by Jetson 20/10/31
=======
        rospy.sleep(0.5)
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        self.preempt_flg = False
<<<<<<< HEAD
        self.act.start()

        self.recognize_tools = RecognizeTools()
        self.recognize_tools.initializeBBox()
=======
        self.find_count = 0
        self.move_count = 0

=======
>>>>>>> 81dc8a8... changed the content of the action　by Laptop 10/12
=======
        self.preempt_flg = False
        self.recognize_tools = RecognizeTools()
>>>>>>> 4c59708... debuged the action by Jetson 20/10/31
=======
>>>>>>> 41ea56a... Fixed third debug by Laptop 11/11
=======
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
=======
        self.preempt_flg = False
<<<<<<< HEAD
        self.recognize_tools = RecognizeTools()
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
=======
>>>>>>> 2b96295... Fixed third debug by Laptop 11/11
        self.act.start()
>>>>>>> e485a97... first commit by Laptop 10/8

        self.recognize_tools = RecognizeTools()
        self.recognize_tools.initializeBBox()

        self.recognize_tools = RecognizeTools()
        self.recognize_tools.initializeBBox()

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        #recognize_tools = RecognizeTools()
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognize_tools.bbox
            object_count, _ = self.recognize_tools.countObject(target_name, bb)
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
=======
        recognize_tools = RecognizeTools()
        move_count = 0
        while not rospy.is_shutdown():
            bb = recognize_tools.bbox
<<<<<<< HEAD
            if target_name in target_dict.keys():
                for i in range(len(target_dict[target_name])):
                    rospy.loginfo(target_dict[target_name][i])
                    object_count, _ = recognize_tools.countObject(target_dict[target_name][i], bb)
                    if bool(object_count):
                        target_name = target_dict[target_name][i]
>>>>>>> a849b3f... first commit by Laptop 10/8
                        break
                    else:
                        # retry
                        rospy.loginfo('There is not object in front.')
<<<<<<< HEAD
                        '''
                        bias = 4
                        object_angle = int(bool(int(object_angle/bias)))*int(object_angle)+int(not int(object_angle/bias))*(object_angle/abs(object_angle))*bias+int(bool(int(object_angle/bias)))*object_angle%(object_angle/abs(object_angle))
                        # ごめんなさい、どうしても一行で書きたかったんです。
                        '''
                        if abs(object_angle) < 5: object_angle=object_angle/abs(object_angle)*5
=======
                        bias = 2
                        object_angle = int(bool(int(x/bias)))*int(x)+int(not int(x/bias))*(x/abs(x))*bias+int(bool(int(x/bias)))*x%(x/abs(x))
                        # ごめんなさい、どうしても一行で書きたかったんです。
>>>>>>> 6148113... The lower limit of the angle was set. 11/19 by Laptop
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                else:
                    #前後進
                    move_count += 1
                    move_range = -0.8*(((move_count)%4)/2)+0.4
                    mimi_control.moveBase(move_range)
                    exist_flg = False
            else:
<<<<<<< HEAD
                find_flg = self.recognize_tools.findObject(target_name)
                exist_flg = find_flg
            action_feedback.recog_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
            rospy.sleep(1.0) #preemptのズレ調整用
            if self.preempt_flg:
                self.preempt_flg = False
                break


if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognize_action = RecognizeAction()
    #recognize_action.recognize_tools.initializeBBox()
=======
        recognize_tools = RecognizeTools()
=======
        #recognize_tools = RecognizeTools()
>>>>>>> 4c59708... debuged the action by Jetson 20/10/31
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognize_tools.bbox
            object_count, _ = self.recognize_tools.countObject(target_name, bb)
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = self.recognize_tools.localizeObject(target_name, bb)
=======
                object_count, _ = recognize_tools.countObject(target_name, bb)
=======
            object_count, _ = recognize_tools.countObject(target_name, bb)
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = recognize_tools.localizeObject(target_name)
>>>>>>> a849b3f... first commit by Laptop 10/8
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_centroid.y += 0.08 # calibrate RealSenseCamera d435
<<<<<<< HEAD
                    object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
=======
                    object_angle = math.atan2(object_centroide.y, object_centroid.x)/math.pi*180
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
=======
        #recognize_tools = RecognizeTools()
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognize_tools.bbox
            object_count, _ = self.recognize_tools.countObject(target_name, bb)
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = self.recognize_tools.localizeObject(target_name, bb)
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_centroid.y += 0.08 # calibrate RealSenseCamera d435
                    object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
                    if abs(object_angle) < 4.5:
                        # success
                        rospy.loginfo('Succeeded')
                        action_result.recog_result = object_centroid
                        self.act.set_succeeded(action_result)
                        break
                    else:
                        # retry
                        rospy.loginfo('There is not object in front.')
                        '''
                        bias = 4
                        object_angle = int(bool(int(object_angle/bias)))*int(object_angle)+int(not int(object_angle/bias))*(object_angle/abs(object_angle))*bias+int(bool(int(object_angle/bias)))*object_angle%(object_angle/abs(object_angle))
                        # ごめんなさい、どうしても一行で書きたかったんです。
                        '''
                        if abs(object_angle) < 5: object_angle=object_angle/abs(object_angle)*5
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                else:
                    #前後進
                    move_count += 1
<<<<<<< HEAD
<<<<<<< HEAD
                    move_range = -0.8*(((move_count)%4)/2)+0.4
                    mimi_control.moveBase(move_range)
=======
                    move_range = -0.4*(((move_count)%4)/2)+0.2
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
=======
                    move_range = -0.8*(((move_count)%4)/2)+0.4
                    mimi_control.moveBase(move_range)
>>>>>>> f412440... Fourth debug by Jetson 11/12
                    exist_flg = False
            else:
<<<<<<< HEAD
<<<<<<< HEAD
                find_flg = self.recognize_tools.findObject(target_name)
                exist_flg = find_flg
            action_feedback.recog_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
<<<<<<< HEAD
<<<<<<< HEAD
            rospy.sleep(1.0) #preemptのズレ調整用
=======
                find_flg = recognize_tools.findObject(target_name)
<<<<<<< HEAD
                ###
                # ここの続きを書く。
                # 見つからなかったときにwhileを抜けるように
                ###
            if loop_flg:
                action_feedback.recog_feedback = exist_flg
                self.act.publish_feedback(action_feedback)
            exist_flg = False
>>>>>>> a849b3f... first commit by Laptop 10/8
=======
                exist_flg = find_flg:
=======
                find_flg = self.recognize_tools.findObject(target_name)
                exist_flg = find_flg
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
            action_feedback.recog_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
>>>>>>> 66eab2a... changed the content of the action　by Laptop 10/12
=======
            #rospy.sleep(1.0) #preemptのズレ調整用
>>>>>>> 2b96295... Fixed third debug by Laptop 11/11
=======
            rospy.sleep(1.0) #preemptのズレ調整用
>>>>>>> f412440... Fourth debug by Jetson 11/12
            if self.preempt_flg:
                self.preempt_flg = False
                break


if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognize_action = RecognizeAction()
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> e485a97... first commit by Laptop 10/8
=======
#    recognize_tools = RecognizeTools()
#    recognize_tools.initializeBBox()
>>>>>>> 4c59708... debuged the action by Jetson 20/10/31
=======
    #recognize_action.recognize_tools.initializeBBox()
>>>>>>> 41ea56a... Fixed third debug by Laptop 11/11
=======
#    recognize_tools = RecognizeTools()
#    recognize_tools.initializeBBox()
>>>>>>> 735e11b... debuged the action by Jetson 20/10/31
=======
    #recognize_action.recognize_tools.initializeBBox()
>>>>>>> 2b96295... Fixed third debug by Laptop 11/11
    rospy.spin()
