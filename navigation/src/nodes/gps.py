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
        """
        Continuously publish GPS data
        """
        rospy.loginfo("GPS node running...")
        # Sending GPS data
        while not rospy.is_shutdown():
            try: 
                # Retrieve GPS data
                coords = self.gps.geo_coords()
                #if coords.lat > 42.2 and coords.lat < 42.3 and coords.lon > -71.7 and coords.lon < -71.9:
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
