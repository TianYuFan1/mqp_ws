#!/usr/bin/env python3

import rospy
import os
from smbus2 import SMBus
import time
from std_msgs.msg import Float32

class Compass:

    def __init__(self):
        """
        Compass class constructor
        """
        # Initialize node
        rospy.init_node("compass")
        # Compass configurations
        self.channel = 1
        self.address = 0x60
        self.reg = 0x02
        self.bus = SMBus(self.channel)
        # Publishers
        self.pub = rospy.Publisher("/compass", Float32, queue_size=1)

    def run(self):
        """
        Continuously publish compass data
        """
        rospy.loginfo("Compass node running...")
        # Sending compass data
        while not rospy.is_shutdown():
            try:
                # Retrieve compass data
                result = self.bus.read_i2c_block_data(self.address, self.reg, 2)
                value = (((result[0] & 0xFF) << 8) | (result[1] & 0xFF)) / 10
                # Publish compass data
                self.pub.publish(value)
                print(value)
            except (ValueError, IOError) as err:
                rospy.loginfo("Compass error")

compass = Compass()
compass.run()