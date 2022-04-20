#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class lidarAnalysis:
    def __init__(self):
        rospy.init_node("lidarAnalysis")
        self.pub = rospy.Publisher("/obstacleStatus", String, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size=1)

    def scanCallback(self, msg):
        front = 0
        range = 90 * math.pi / 180
        angleStart = front - range / 2
        indexStart = int((angleStart - msg.angle_min) / msg.angle_increment)
        angleEnd = front + range / 2
        indexEnd = int((angleEnd - msg.angle_min) / msg.angle_increment)
        lidarReadings = msg.ranges[indexStart:indexEnd]
        minDist = min(lidarReadings)
        # print("min dist", minDist)
        if minDist < 1:
            self.pub.publish("blocked")
        else:
            self.pub.publish("clear")
    
    def run(self):
        rospy.spin()

lidarAnalyzer = lidarAnalysis()
lidarAnalyzer.run()
