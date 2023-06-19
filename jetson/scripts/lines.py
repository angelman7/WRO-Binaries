#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import Int8
from modules.color import Color

direction = 0

#{"data": "orange", "hmin": 19, "hmax": 32, "smin": 50, "smax": 150, "vmin": 70, "vmax": 150}
#{"data": "blue", "hmin": 85, "smin": 85, "vmin": 65, "hmax": 92, "smax": 108, "vmax": 87}

def talker():
    global direction

    time.sleep(3)
    color = Color()
    pub = rospy.Publisher('line', Int8, queue_size=1)
    rospy.init_node('lines', anonymous=False)
    rate = rospy.Rate(400) # 10hz
    current_time = 0
    last_time = 0
    while not rospy.is_shutdown():
        current_time = time.time()
        line_d = color.detect_line()
        # line_d = color.read_hsv()
        if line_d != 0 and (line_d == direction or direction == 0) and current_time - last_time > 2:
            direction = line_d
            last_time = current_time
            rospy.loginfo(str(line_d))
            pub.publish(line_d)

        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass