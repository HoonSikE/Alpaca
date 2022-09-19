#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2, degrees, radians
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber('local_path', Path, self.path_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=2)


        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=2

        self.is_path=False
        self.is_odom=False

        self.is_look_forward_point=False

        self.forward_point=[0,0,0]
        self.current_postion=Point()

        self.vehicle_length = 2.75
        self.lfd = 15


        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True:

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                    self.ctrl_cmd_msg.velocity = 40.0
                    print(self.ctrl_cmd_msg.steering)
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                #TODO: (4) 제어입력 메세지 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):        
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        _,_,self.vehicle_yaw = tf.transformations.euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y


    def calc_pure_pursuit(self,):                
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]
        
        #TODO: (2) 좌표 변환 행렬 생성
        theta = - self.vehicle_yaw

        trans_matrix = np.array([[cos(theta), -sin(theta), 0], 
                                 [sin(theta),  cos(theta), 0], 
                                 [     0    ,      0     , 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for i in self.path.poses:
            position = i.pose.position
            path_point = trans_matrix.dot([position.x - vehicle_position.x, position.y - vehicle_position.y, 1])
            
            
            
            if path_point[0] > 0:
                # print(path_point[:2])
                dis = ((path_point[0])**2 + (path_point[1])**2)**0.5
            
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (3) Steering 각도 계산
        x = abs(self.forward_point[0])
        y = abs(self.forward_point[1])
        print(self.forward_point)
        distance = (x**2 + y**2)**0.5
        theta = atan2(y , x)
        # print(theta, degrees(theta))
        pure_pursuit_radian = atan2(2*self.vehicle_length*sin(theta) , distance)
        # print(degrees(pure_pursuit_radian))
        # quit()
        return - pure_pursuit_radian if self.forward_point[1] < 0 else pure_pursuit_radian



if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass