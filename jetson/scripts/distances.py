#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int16MultiArray
from modules.distance_sensors import Distances

distances_array = Int16MultiArray()

def talker():
    distances = Distances()
    pub = rospy.Publisher('lrdistance', Int16MultiArray, queue_size=1)
    rospy.init_node('distances', anonymous=False)
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        distances_array.data = [distances.left(), distances.right()]        
        rospy.loginfo(str(distances_array.data))
        pub.publish(distances_array)
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass