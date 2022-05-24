#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Float32, Float64
from std_msgs.msg import Int16
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point

import math

class pointDriver:
    def __init__(self):
        rospy.init_node("pointDriver")
        rospy.Subscriber("/compass", Float32, self.headingCallback, queue_size=1)
        rospy.Subscriber("/gps", NavSatFix, self.gpsCoordsCallback, queue_size=1)
        rospy.Subscriber("/gps/setpoint", Point, self.goalCallback, queue_size=1)
        rospy.Subscriber("/robotStatus", String, self.statusCallback, queue_size=1)
        rospy.Subscriber("/obstacleStatus", String, self.statusObsCallback, queue_size=1)
        rospy.Subscriber("/turn_angle", Float32, self.turnCallback, queue_size=1)

        self.pubRight1 = rospy.Publisher("/vesc_r1/commands/motor/duty_cycle", Float64, queue_size=1)
        self.pubRight2 = rospy.Publisher("/vesc_r2/commands/motor/duty_cycle", Float64, queue_size=1)
        self.pubLeft1  = rospy.Publisher("/vesc_l1/commands/motor/duty_cycle", Float64, queue_size=1)
        self.pubLeft2  = rospy.Publisher("/vesc_l2/commands/motor/duty_cycle", Float64, queue_size=1)

        self.pth = 0.0
        self.gth = 0.0

        self.lat = 0
        self.lon = 0
        
        self.turnAngle = 0
        self.goal = None
        self.status = "stopped"
        self.statusObs = "blocked"
    
    def turnCallback(self, msg):
        self.turnAngle = msg.data

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
        self.goal = [0,0]
        self.goal[0] = msg.x
        self.goal[1] = msg.y
    
    def statusCallback(self, msg):
        self.status = msg.data

    def statusObsCallback(self, msg):
        self.statusObs = msg.data

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
        # if self.status == "drive" and self.statusObs == "clear":
        if self.status == "drive":
            self.pubRight1.publish(-rSpeed)
            self.pubRight2.publish(-rSpeed)
            self.pubLeft1.publish(lSpeed)
            self.pubLeft2.publish(lSpeed)
            print(rSpeed, lSpeed)
        else:
            self.pubRight1.publish(0)
            self.pubRight2.publish(0)
            self.pubLeft1.publish(0)
            self.pubLeft2.publish(0)
            # print("stopped")
    
    def run(self):
        linspeed = 0.1
        aspeed = 0.08
        while not rospy.is_shutdown():
            if self.goal is None or self.lat == 0 or self.lon == 0:
                continue
            self.angleToGoal()
            # print(self.turnAngle)
            # if abs(self.getBestAngle(self.gth)) > 5:  
            if abs(self.turnAngle) > 10:  
                # self.rotate(self.turn_angle, aspeed)
                if self.turnAngle < 0:
                    self.sendSpeeds(aspeed,-aspeed)
                else:
                    self.sendSpeeds(-aspeed,aspeed)
            else:
                self.drive(-linspeed)
            rospy.sleep(.01)

driver = pointDriver()
driver.run()
