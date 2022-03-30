#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from ublox_gps import UbloxGps
import serial

class GPS:

    def __init__(self):
        """
        GPS class constructor
        """
        # Initialize node
        rospy.init_node("gps")
        # GPS configurations
        self.port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        self.gps = UbloxGps(self.port)
        # Publishers
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=1)
    
    def run(self):
        rospy.loginfo("GPS node running...")
        while not rospy.is_shutdown():
            # Sending GPS data
            try: 
                # Retrieve GPS data
                coords = self.gps.geo_coords()
                # Publish GPS data
                gps_msg = NavSatFix()
                gps_msg.header.stamp = rospy.Time.now()
                gps_msg.header.frame_id = "gps"
                gps_msg.latitude = coords.lat
                gps_msg.longitude = coords.lon
                self.gps_pub.publish(gps_msg)
            # GPS error
            except (ValueError, IOError) as err:
                rospy.loginfo("GPS error")

gps = GPS()
gps.run()
