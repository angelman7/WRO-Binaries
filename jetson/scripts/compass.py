#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int16
import math
import time

t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from modules.mpu9250_mag_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue

bias ={'x': 12.0849609375, 'y': 14.208984375, 'z': 47.900390625}
scale = {'x': 0.966996699669967, 'y': 1.0316901408450705, 'z': 1.0034246575342465}

def fix_xyz(x, y, z):
    x = scale["x"] * (x - bias['x']) 
    y = scale["y"] * (y - bias['y'])
    z = scale["z"] * (z - bias['z'])
    return x, y, z

def heading():
    mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
    mx,my, mz = fix_xyz(mx, my, mz)
    bearing  = math.atan2(my, mx) 
    if (bearing < 0):
        bearing += 2 * math.pi
    return int(math.degrees(bearing))

def talker():
    pub = rospy.Publisher('heading', Int16, queue_size=10)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        
        h = heading()
        msg = int(h)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass