#!/usr/bin/python3
import time
import serial
import rospy
from std_msgs.msg import Int32MultiArray, Int8, String
import os

cmd = 'sudo chmod 666 /dev/ttyTHS1'
os.system(cmd)


MAIN_ROT_TICKS = 400
LEFT_ROT_TICKS = 40
RIGHT_ROT_TICKS = 40
WHEEL_PERIMETER = 196
MAX_SPEED_RPM = 1000

def _map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


serial_port = serial.Serial(
                port="/dev/ttyTHS1",
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )

main_enc = left_enc = right_enc = 1
encoders_array = Int32MultiArray()
# encoders_array.data_length = 3

# Wait a second to let the port initialize
time.sleep(1)
speed = 0

def read_encoders_fake():
    global main_enc, left_enc, right_enc

    main_enc = left_enc = right_enc = 10


def read_encoders():
    global main_enc, left_enc, right_enc
    # rospy.loginfo("in read")
    data = "e".encode()   
    serial_port.write(data)
    while serial_port.inWaiting() == 0:
        pass
        # print("Waiting for message")
    c = ""
    while serial_port.inWaiting() > 0 and c != 'M':
        c = serial_port.read().decode()
    data = ""
    while serial_port.inWaiting() > 0:
        c = serial_port.read().decode()
        if c == 'L' and data != '':
            main_enc = int(data)
            break
        data += c
    data = ""
    while serial_port.inWaiting() > 0:
        c = serial_port.read().decode()
        if c == 'R' and data != '':
            left_enc = int(data)
            break
        data += c
    data = ""
    while serial_port.inWaiting() > 0:
        c = serial_port.read().decode()
        if c == '#' and data != '':
            right_enc = int(data)
            break
        data += c

def speed_callback(data):
    global speed

    rospy.loginfo(rospy.get_caller_id() + "Speed %s", data.data)
    speed = _map(int(data.data), -100, 100, -MAX_SPEED_RPM, MAX_SPEED_RPM)
    data = ("#" + str(speed) + "_").encode()
    # print(data)
    serial_port.write(data)
    

def talker():
    global main_enc, left_enc, right_enc, encoders_array, serial_port

    pub = rospy.Publisher('encoders', Int32MultiArray, queue_size=10)
    rospy.init_node('dc', anonymous=False)
    rate = rospy.Rate(50) 
    rospy.Subscriber("speed", String, speed_callback)
    time.sleep(1)
    serial_port.write(0)
    time.sleep(1)
    while not rospy.is_shutdown():
        try:
            read_encoders()
        except:
            rospy.loginfo("Serial read exception, wait to connect again")
            serial_port = serial.Serial(
                port="/dev/ttyTHS1",
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(1)
            main_enc = left_enc = right_enc = 0
            encoders_array = Int32MultiArray()
            continue
        # read_encoders_fake()
        encoders_array.data = [left_enc, main_enc, right_enc]
        rospy.loginfo(str(encoders_array.data))
        pub.publish(encoders_array)
        rate.sleep()

talker()