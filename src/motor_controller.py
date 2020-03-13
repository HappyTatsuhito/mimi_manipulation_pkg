#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import math
import threading
import time
# ros msgs
from std_msgs.msg import Bool,Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
# ros srvs
from dynamixel_workbench_msgs.srv import DynamixelCommand
from manipulation.srv import ManipulateSrv

class MotorController(object):
    def __init__(self):
        # ROS Topic Subscriber
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,self.getMotorStateCB)
        # ROS Topic Publisher
        self.motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
        # ROS Service Client
        self.motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        # Motor Parameters
        self.origin_angle = [1870, 2275, 1890, 2048, 3500, 1800]
        self.current_pose = [0]*5
        self.torque_error = [0]*5
        self.rotation_velocity = [0]*5
        
    def getMotorStateCB(self, state):
        for i in range(5):
            self.current_pose[i] = state.dynamixel_state[i].present_position
            self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
            self.torque_error[i] = state.dynamixel_state[i].present_current

    def callMotorService(self, motor_id, rotate_value):
        if type(rotate_value) == type(float()):
            rotate_value = self.radToStep(rotate_value)
        res = self.motor_client('', motor_id, 'Goal_Position', rotate_value)

    def radToStep(self,rad):
        return int((rad + math.pi) / (2*math.pi) * 4095)

    def stepToRad(self,step):
        return step / 4095.0 * 2*math.pi - math.pi
    

class JointController(MotorController):
    def __init__(self):
        super(JointController,self).__init__()
        # ROS Topic Subscriber
        rospy.Subscriber('/servo/shoulder',Float64,self.shoulderPub)
        rospy.Subscriber('/servo/elbow',Float64,self.elbowPub)
        rospy.Subscriber('/servo/wrist',Float64,self.wristPub)
        rospy.Subscriber('/servo/endeffector',Bool,self.endeffectorPub)
        rospy.Subscriber('/servo/head',Float64,self.headPub)

    def shoulderPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        step = self.radToStep(rad)
        step0 = 4095 - step + (self.origin_angle[0]-2048)
        step1 = step + (self.origin_angle[1]-2048)
        print '0:', step0, ' 1:', step1
        thread_m0 = threading.Thread(target=self.callMotorService, args=(0, step0,))
        thread_m1 = threading.Thread(target=self.callMotorService, args=(1, step1,))
        thread_m0.start()
        thread_m1.start()
        rospy.sleep(0.2)
        while (self.rotation_velocity[0] > 0 or self.rotation_velocity[1] > 0) and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[0]) > 100 or abs(self.torque_error[1] > 100):
            thread_m0 = threading.Thread(target=self.callMotorService, args=(0, self.current_pose[0],))
            thread_m1 = threading.Thread(target=self.callMotorService, args=(1, self.current_pose[1],))
            thread_m0.start()
            thread_m1.start()

    def elbowPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        rad *= -1
        step = self.radToStep(rad) + (self.origin_angle[2]-2048)
        print '2: ', step
        self.callMotorService(2, step)
        rospy.sleep(0.2)
        while self.rotation_velocity[2] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[2]) > 100:
            self.callMotorService(2, self.current_pose[2])

    def wristPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        step = self.radToStep(rad) + (self.origin_angle[3]-2048)
        print '3: ', step
        self.callMotorService(3, step)
        rospy.sleep(0.2)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[3]) > 100:
            self.callMotorService(3, self.current_pose[3])

    def endeffectorPub(self,req):
        angle = self.origin_angle[4]
        self.callMotorService(4, angle)
        while self.rotation_velocity[4] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        grasp_flg = True
        while abs(self.torque_error[4]) <= 50 and not rospy.is_shutdown():
            angle -= 30
            self.callMotorService(4, angle)
            rospy.sleep(0.06)
            while self.rotation_velocity[4] > 5.0 and not rospy.is_shutdown():
                pass
            if angle < 2850:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        self.callMotorService(4, self.current_pose[4]-60)
        print 'fin'
        return grasp_flg

    def headPub(self,rad):
        if type(rad) == type(Float64()):
            rad = rad.data
        step = self.radToStep(rad) + (self.origin_angle[5]-2048)
        print '5: ', step
        self.callMotorService(5, step)
    
    
class ArmPoseChanger(JointController):
    def __init__(self):
        super(ArmPoseChanger,self).__init__()
        # ROS Topic Subscriber
        arm_changer = rospy.Service('/servo/arm', ManipulateSrv, self.changeArmPose)

    def inverseKinematics(self, x, y):
        l0 = 0.78# Height from ground to shoulder(metre)
        l1 = 0.22# Length from shoulder to elbow(metre)
        l2 = 0.17# Length from elbow to wrist(metre)
        l3 = 0.15# Length of end effector(metre)
        x -= l3
        y -= l0
        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)
        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
            wrist_angle = -1*(shoulder_angle + elbow_angle)
            shoulder_angle *= 2.1
            elbow_angle *= 2.1
            #self.armCotnroller(shoulder_angle, elbow_angle, wrist_angle)
            return [shoulder_angle, elbow_angle, wrist_angle]
        except ValueError:
            rospy.loginfo('I can not move arm.')
            return False
        
    def armController(self, shoulder_param, elbow_param, wrist_param):
        thread_shoulder = threading.Thread(target=self.shoulderPub, args=(shoulder_param,))
        thread_elbow = threading.Thread(target=self.elbowPub, args=(elbow_param,))
        thread_wrist = threading.Thread(target=self.wristPub, args=(wrist_param,))
        thread_wrist.start()
        rospy.sleep(0.2)
        thread_elbow.start()
        rospy.sleep(0.2)
        thread_shoulder.start()

    def changeArmPose(self, cmd):
        if type(cmd) != str:
            cmd = cmd.target
        rospy.loginfo('Chagnge arm command : %s'%cmd)
        if cmd == 'origin':
            self.originMode()
            return True
        elif cmd == 'carry':
            self.carryMode()
            return True
        elif cmd == 'give':
            self.giveMode()
            return True
        elif cmd == 'place':
            self.placeMode()
            return True
        else :
            rospy.loginfo('No sudh change arm command.')
            return False

    def originMode(self):
        shoulder_param = 0.0
        elbow_param = 0.0
        wrist_param = 0.0
        self.armController(shoulder_param, elbow_param, wrist_param)
        
    def carryMode(self):
        shoulder_param = -3.0
        elbow_param = 2.8
        wrist_param = 1.5
        self.armController(shoulder_param, elbow_param, wrist_param)

    def giveMode(self):
        shoulder_param = -1.2
        elbow_param = 2.6
        wrist_param = -0.7
        self.armController(shoulder_param, elbow_param, wrist_param)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(2.0)
        wrist_error = abs(self.torque_error[3])
        give_time = time.time()
        while abs(wrist_error - abs(self.torque_error[3])) < 10 and time.time() - give_time < 5.0 and not rospy.is_shutdown():
            pass
        self.callMotorService(4, self.origin_angle[4])
        self.carryMode()

    def placeMode(self):
        #現時点では家具の高さを予めプログラムに打ち込む必要があり、
        #その情報をobject_grasperに格納しているのでそちらでplaceの関数を再構築しています。
        pass

        
if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = ArmPoseChanger()
    rospy.spin()
