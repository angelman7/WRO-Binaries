#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int16
import math
import time

time.sleep(6)
t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from modules.mpu9250_gyro_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue
# -2.5337066650390625, 1.4799346923828125, -1.128387451171875
# gyro_offsets: [-2.4185638427734375, 1.435003662109375, -1.0225906372070312]
z_offset = -1.0158615112304688#-1.001710669921875


def talker():
    global z_offset

    pub = rospy.Publisher('gyro_angle', Int16, queue_size=10)
    rospy.init_node('gyro', anonymous=False)
    rate = rospy.Rate(100) 
    
    previous_time = time.time() * 1000
    current_time = time.time() * 1000
    rospy.loginfo("Start calibration")
    z_offset = calibrate_gyro(count=700)
    rospy.loginfo("End of calibration, z_offset: " + str(z_offset))
    current_angle = 0
    while not rospy.is_shutdown():
        
        wz = mpu6050_gyro() # read and convert mpu6050 data
        current_time = time.time() * 1000
        current_angle += (wz - z_offset)* ((current_time - previous_time)/1000)
        # h = heading()
        # print(wz)
        previous_time = current_time
        rospy.loginfo(str(current_angle))
        pub.publish(int(current_angle))
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass