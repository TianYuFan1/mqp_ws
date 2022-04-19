#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Int16
from sensor_msgs.msg import NavSatFix

import math

class pointDriver:
    def __init__(self):
        rospy.init_node("pointDriver")
        rospy.Subscriber("/compass", Float32, self.headingCallback, queue_size=1)
        rospy.Subscriber("/gps", NavSatFix, self.gpsCoordsCallback, queue_size=1)
        rospy.Subscriber("/gps/setpoint", NavSatFix, self.goalCallback, queue_size=1)
        self.pubRight1 = rospy.Publisher("/vesc_r1/commands/motor/speed", Float64, queue_size=1)
        self.pubRight2 = rospy.Publisher("/vesc_r2/commands/motor/speed", Float64, queue_size=1)
        self.pubLeft1  = rospy.Publisher("/vesc_l1/commands/motor/speed", Float64, queue_size=1)
        self.pubLeft2  = rospy.Publisher("/vesc_l2/commands/motor/speed", Float64, queue_size=1)

        self.pth = 0.0
        self.gth = 0.0

        self.lat = 0
        self.lon = 0
        
        self.goal = [20, -72]

    def getBestAngle(self, targetAngle):
        dth = targetAngle - self.pth
        if dth < -180:
            dth = 360 + dth
        elif dth > 180:
            dth = -180 + dth
        return dth

    def headingCallback(self, msg):
        # print(msg.data)
        self.pth = msg.data
    
    def gpsCoordsCallback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def goalCallback(self, msg):
        self.goal[0] = msg.latitude
        self.goal[1] = msg.longitude

    def rotate(self, angle, aspeed):
        dir = self.getBestAngle(angle)
        if dir < 0:
            self.sendSpeeds(aspeed,-aspeed)
            # print("turning left")
        else:
            self.sendSpeeds(-aspeed,aspeed)
            # print("turning right")
        rospy.sleep(.02)
    
    def drive(self, linSpeed):
        self.sendSpeeds(linSpeed,linSpeed)
        # print("driving straight")
        # print(self.pth)

    def angleToGoal(self):
        self.gth = math.degrees(math.atan2(self.goal[0] - self.lat, self.goal[1] - self.lon))-90
        # print(self.gth, self.goal, self.lat, self.lon)
    
    def sendSpeeds(self, rSpeed, lSpeed):
        self.pubRight1.publish(rSpeed)
        self.pubRight2.publish(rSpeed)
        self.pubLeft1.publish(lSpeed)
        self.pubLeft2.publish(lSpeed)
        print(rSpeed, lSpeed)
    
    def run(self):
        linspeed = 20000
        aspeed = 10000
        while not rospy.is_shutdown():
            self.angleToGoal()
            print(self.getBestAngle(self.gth), self.goal)
            if abs(self.getBestAngle(self.gth)) > 20:  
                self.rotate(self.gth, aspeed)
            else:
                self.drive(linspeed)
            rospy.sleep(.01)

driver = pointDriver()
driver.run()
