#!/usr/bin/env python3
import rospy, time
import termios, sys, tty
from std_msgs.msg import String, Int16, Int16MultiArray, Int32MultiArray, Int8
import steering
from modules.pid import PID

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
compass_pos = 0
first_line = 0
do_turn = False
tics_per_mm = 5.2
steer = steering.Steering()
reference_angle = 0

pub = rospy.Publisher('speed', String, queue_size=10)

def line_callback(data):
    global first_line, do_turn

    # rospy.loginfo(rospy.get_caller_id() + "Line %s", data.data)
    if first_line == 0:
        first_line = data.data
    if first_line == data.data and do_turn is False:
        do_turn = True
        print("$$$$$$$$$$$ TURN $$$$$$$$$$$$$$")

def reset_gyro_angle():
    global reference_angle

    reference_angle = gyro_pos

def read_angle():
    return reference_angle - gyro_pos



def distance_callback(data):
    global left_distance, right_distance

    # rospy.loginfo(rospy.get_caller_id() + "Distances %s", data.data)
    left_distance = data.data[0]
    # right_distance = data.data[1]
    
    # print("LD: ", left_distance)
    # print("RD: ", right_distance)


def gyro_callback(data):
    global gyro_pos

    # rospy.loginfo(rospy.get_caller_id() + "Gyro angle %s", str(data.data))
    gyro_pos = data.data

def heading_callback(data):
    global compass_pos

    # rospy.loginfo(rospy.get_caller_id() + "Heading %s", data.data)
    compass_pos = data.data

def encoders_callback(data):
    global encoder

    # rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
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
    
def drive_straight(speed=50, pos=0, end="turns", value=12, pid_ks=(1.5, 0.1, 0.005)):
    '''
    :params:
    -end: ["turns", "distance"]
    '''
    # Initialise PID object
    pid = PID(pid_ks[0], pid_ks[1], pid_ks[2])
    pid.SetPoint = 0
    pid.setSampleTime(0.005)
    dir_factor = 1
    if speed < 0:
        dir_factor = -1
    end_tics = 0
    starting_tics = encoder.data[1] 
    if end == "distance":
        end_tics = value * tics_per_mm
    dc_speed(speed)
    start_time = time.time()
    turns = 0
    last_turn = 0
    
    while True:
        current_ms = time.time() * 1000
        # Calculate error
        error = read_angle() + pos
        # Calculate the correction value with the PID object
        pid.update(dir_factor * error)
        u = int(pid.output)
        current_steer = int(min(100, max(u, -100)))
        # if 0 < current_steer < 30:
        #      current_steer = 30
        # print(f'current_steer: {current_steer}')
        steer.set_steering(current_steer)
        time_to_stop = 0
        
        # if time.time() - start_time > 10:
        #     dc_speed(0)
        #     break
        if left_distance > 600 and current_ms - last_turn > 3000:
            turns += 1
            # dc_speed(0)
            last_turn = current_ms
            pos += 90
            print(f'\nLeft Distance: {left_distance}', end='\t')
            print(f'current_steer: {current_steer}', end='\t')
            print(f'pos: {pos}', end='\t')
            print(f'Turns: {turns}', end='\t')
        
        if time_to_stop == 0:
            print(1)
            if end == "turns" and turns == value:
                print(2)
                time_to_stop = time.time()
        elif time.time() - time_to_stop > 5:
            print(3)
            dc_speed(0)
            steer.set_steering(0)
            break
        # elif end == "distance" and encoder.data[1] - starting_tics >= end_tics:
        #     print(8)
        #     dc_speed(0)
        #     steer.set_steering(0)
        #     break

def drive_time(speed=30, ms=5):
    lstart = encoder.data[0]
    rstart = encoder.data[2]
    cstart = encoder.data[1]
    steer.set_steering(0)
    time.sleep(5)
    dc_speed(30)
    start_secs = time.time() 
    while time.time() - start_secs < ms:
        pass
    dc_speed(0)
    print("lstart:", encoder.data[0])
    print("rstart:", encoder.data[2])
    print("cstart:", encoder.data[1])
    
def drive_mm(speed=50, distance_mm=1000, steering=0):
    steer.set_steering(steering)
    cstart = encoder.data[1]
    distance_tics = distance_mm * tics_per_mm
    dc_speed(speed)
    while encoder.data[1] - cstart < distance_tics:
        pass
    dc_speed(0)

def listener():
    rospy.init_node('run1', anonymous=False)

    rospy.Subscriber("line", Int8, line_callback)
    rospy.Subscriber("lrdistance", Int16MultiArray, distance_callback)
    rospy.Subscriber("gyro_angle", Int16, gyro_callback)
    rospy.Subscriber("heading", Int16, heading_callback)
    rospy.Subscriber("encoders", Int32MultiArray, encoders_callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    rate = rospy.Rate(100) 
    dc_speed(0)
    time.sleep(3)

    # drive_time()
    # drive_mm(speed=50, distance_mm=1000, steering=0)
    # while not rospy.is_shutdown():
    try:
        reset_gyro_angle()
        #     Normal speed = 60
        drive_straight(speed=60, pos=0, end="turns", value=4, pid_ks=(3.5, 0.1, 0.005))
        # time.sleep(3)
        # drive_straight(speed=-70, pos=0, end="distance", value=1000, pid_ks=(3, 0.1, 0.005))
        
        dc_speed(0)
        time.sleep(1)

    except KeyboardInterrupt:
        dc_speed(0)
        steer.set_steering(0)
        time.sleep(1)

    
if __name__ == '__main__':
    listener()