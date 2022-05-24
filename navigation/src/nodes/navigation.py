#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from search import Search
from geometry_msgs.msg import Point

class Navigation:

    def __init__(self):
        """
        Navigation class constructor
        """
        # Initialize node
        rospy.init_node("navigation")
        # Navigation configurations
        self.heading = None
        self.location = None  # lon, lat
        self.goal = None
        # Subscribers
        rospy.Subscriber("/gps", NavSatFix, self.gps_cb)
        rospy.Subscriber("/compass", Float32, self.compass_cb)
        rospy.Subscriber("/goal", Point, self.goal_cb)
        self.pubSetpoint = rospy.Publisher("/gps/setpoint", Point, queue_size=1)
        self.pubTurnAngle = rospy.Publisher("/turn_angle", Float32, queue_size=1)
        rospy.sleep(0.5)
    
    def gps_cb(self, msg):
        self.location = [msg.longitude, msg.latitude]
    
    def compass_cb(self, msg):        
        self.heading = msg.data

    def goal_cb(self, msg):
        self.goal = [msg.x, msg.y]


    def dest_angle(self, start, end):
        lon_diff = end[0] - start[0]
        lat_diff = end[1] - start[1]
        angle = math.atan2(lon_diff, lat_diff)
        angle = math.degrees(angle)
        return angle

    def dest_distance(self, start, end):
        R = 6378.137
        lon_diff = end[0] * math.pi / 180 - start[0] * math.pi / 180
        lat_diff = end[1] * math.pi / 180 - start[1] * math.pi / 180
        a = math.sin(lat_diff/2) * math.sin(lat_diff/2) + math.cos(end[0] * math.pi / 180) * math.cos(end[1] * math.pi / 180) * math.sin(lon_diff/2) * math.sin(lon_diff/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c
        return d * 1000; # meters
        
    def run(self):
        # start = [-71.8077749, 42.2744825]
        # self.goal = [-71.8086798, 42.274100399999995]

        search = Search()
        # path = search.run(start, self.goal)

        # current_waypoint_index = 0
        # current_waypoint = path[current_waypoint_index]

        while not rospy.is_shutdown():
            if self.location is None or (self.location[0] == 0 and self.location[1] == 0):
                print("Not fixed")
            elif self.heading is None:
                print("No heading")
            elif self.goal is None:
                print("No goal")
            else:
                path = search.run(self.location, self.goal)
                if len(path) > 2:
                    waypoint = path[1]
                else:
                    waypoint = path[0]

                if (self.dest_distance(self.location, self.goal) < 3):
                        print("Arrived at final destination")                    
                        break
                else:
                    # Calculate heading correction
                    loc_angle_diff = self.dest_angle(self.location, waypoint.coord) # Angle between start and waypoint from North
                    angle_travel = loc_angle_diff - self.heading

                    while angle_travel < -180:
                        angle_travel += 360
                    
                    while angle_travel > 180:
                        angle_travel -= 360

                    angle = Float32()
                    angle.data = angle_travel

                    self.pubTurnAngle.publish(angle)

                    print(angle_travel)
                    print(self.dest_distance(self.location, waypoint.coord))
                    print("Not within area")
                    
                    msg = Point()
                    msg.y = waypoint.coord[1]
                    msg.x = waypoint.coord[0]
                    msg.z = 0
                    self.pubSetpoint.publish(msg)
            rospy.sleep(0.5)

nav = Navigation()
nav.run()
