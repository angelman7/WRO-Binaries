#!/usr/bin/env python3
from simple_pid import PID
import rospy, time
import termios, sys, tty
from std_msgs.msg import String, Int16, Int16MultiArray, Int8
import steering

encoder = Int16MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
compass_pos = 0
first_line = 0
do_turn = False

steer = steering.Steering()

pub = rospy.Publisher('speed', String, queue_size=10)

def line_callback(data):
    global first_line, do_turn

    rospy.loginfo(rospy.get_caller_id() + "Line %s", data.data)
    if first_line == 0:
        first_line = data.data
    if first_line == data.data and do_turn is False:
        do_turn = True
        print("$$$$$$$$$$$  TURN $$$$$$$$$$$$$$")



def distance_callback(data):
    global left_distance, right_distance

    rospy.loginfo(rospy.get_caller_id() + "Distances %s", data.data)
    left_distance = data.data[0]
    right_distance = data.data[1]
    # print("LD: ", left_distance)
    # print("RD: ", right_distance)


def gyro_callback(data):
    global gyro_pos

    rospy.loginfo(rospy.get_caller_id() + "Gyro angle %s", str(data.data))
    gyro_pos = data.data

def heading_callback(data):
    global compass_pos

    rospy.loginfo(rospy.get_caller_id() + "Heading %s", data.data)
    compass_pos = data.data

def encoders_callback(data):
    global encoder

    rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

    
def dc_speed(speed=0):
    msg = str(speed)
    rospy.loginfo(msg)
    pub.publish(msg)

def turn(angle):
    global do_turn

    print("start steering in demo turn function")
    time.sleep(0.5)
    print("stop steering in demo turn function")
    do_turn = False
    


def listener():
    
    rospy.init_node('run1', anonymous=True)

    rospy.Subscriber("line", Int8, line_callback)
    rospy.Subscriber("lrdistance", Int16MultiArray, distance_callback)
    rospy.Subscriber("gyro_angle", Int16, gyro_callback)
    rospy.Subscriber("heading", Int16, heading_callback)
    rospy.Subscriber("encoders", Int16MultiArray, encoders_callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    rate = rospy.Rate(100) 

    try:
        filedescriptors = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
        speed = 0
        current_steer = 0
        while not rospy.is_shutdown():
            key = sys.stdin.read(1)[0]
            if key == 'w':
                speed=min(speed+10, 100)
                dc_speed(speed)
            if key == 's':
                speed=max(speed-10, -100)
                dc_speed(speed)
            if key == 'a':
                current_steer = max(current_steer-10, -100)
                steer.set_steering(current_steer)
            if key == 'd':
                current_steer = min(current_steer+10, 100)
                steer.set_steering(current_steer)
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
if __name__ == '__main__':
    listener()