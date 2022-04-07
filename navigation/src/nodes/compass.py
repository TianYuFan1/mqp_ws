# #!/usr/bin/env python3

# import rospy
# import os
# from smbus2 import SMBus
# import time
# from std_msgs.msg import Float32

# class Compass:

#     def __init__(self):
#         """
#         Compass class constructor
#         """
#         # Initialize node
#         rospy.init_node("compass")
#         # Compass configurations
#         self.channel = 1
#         self.address = 0x60
#         self.reg = 0x02
#         self.bus = SMBus(self.channel)
#         # Publishers
#         self.pub = rospy.Publisher("/compass", Float32, queue_size=1)

#     def run(self):
#         """
#         Continuously publish compass data
#         """
#         rospy.loginfo("Compass node running...")
#         # Sending compass data
#         while not rospy.is_shutdown():
#             try:
#                 # Retrieve compass data
#                 result = self.bus.read_i2c_block_data(self.address, self.reg, 2)
#                 value = (((result[0] & 0xFF) << 8) | (result[1] & 0xFF)) / 10
#                 # Publish compass data
#                 self.pub.publish(value)
#             except (ValueError, IOError) as err:
#                 rospy.loginfo("Compass error")

# compass = Compass()
# compass.run()

from mpu9250_i2c import *
from fusion import Fusion


time.sleep(1) # delay necessary to allow mpu9250 to settle

fuse = Fusion()

print('recording data')
while 1:
    try:
        ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
    except:
        continue
    
    print('{}'.format('-'*30))
    print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax,ay,az))
    print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx,wy,wz))
    print('mag [uT]:   x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(mx,my,mz))
    print('{}'.format('-'*30))

    fuse.update((ax,ay,az), (wx,wy,wz), (mx,my,mz))
    print(fuse.heading)
    time.sleep(.1)