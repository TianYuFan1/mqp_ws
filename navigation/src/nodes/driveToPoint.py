#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from MQP.msg import gpsCoords
import math

class pointDriver:
    def __init__(self):
        rospy.init_node("pointDriver")
        rospy.Subscriber("heading", Float32, self.headingCallback, queue_size=1)
        rospy.Subscriber("/gps", gpsCoords, self.gpsCoordsCallback, queue_size=1)
        rospy.Subscriber("/gps/setpoint", gpsCoords, self.goalCallback, queue_size=1)
        self.pubRight = rospy.Publisher("Right", Int16, queue_size=1)
        self.pubLeft = rospy.Publisher("Left", Int16, queue_size=1)

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
        self.lat = msg.lat
        self.lon = msg.lon

    def goalCallback(self, msg):
        self.goal[0] = msg.lat
        self.goal[1] = msg.lon

    def rotate(self, angle, aspeed):
        dir = self.getBestAngle(angle)
        if dir < 0:
            self.pubRight.publish(aspeed)
            self.pubLeft.publish(-aspeed)
            # print("turning left")
        else:
            self.pubRight.publish(-aspeed)
            self.pubLeft.publish(aspeed)
            # print("turning right")
        rospy.sleep(.02)
    
    def drive(self, linSpeed):
        self.pubRight.publish(linSpeed)
        self.pubLeft.publish(linSpeed)
        # print("driving straight")
        # print(self.pth)

    def angleToGoal(self):
        self.gth = math.degrees(math.atan2(self.goal[0] - self.lat, self.goal[1] - self.lon))-90
        # print(self.gth, self.goal, self.lat, self.lon)
    
    def run(self):
        linspeed = 30
        aspeed = 15
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