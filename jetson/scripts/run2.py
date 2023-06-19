#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import String, Int16, Int16MultiArray, Int32MultiArray, Int8
import steering
from modules.pid import PID
from modules.button import Button
import RPi.GPIO as GPIO

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
# pillar_array.data = ["color", "cx", "cy", "height", "center"]
pillar = Int16MultiArray()
pillar.data = [0, 0, 0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
line = 0
state = 0
steer = steering.Steering()
reference_angle = 0
gyro_target = 0
pub = rospy.Publisher('speed', String, queue_size=10)


def change_state(channel):
    global state

    if state == 0:
        state = 1
        rospy.loginfo("State changed to 1")
    elif state == 1:
        state = -1
        rospy.loginfo("State changed to -1")
    

button = Button(pin=21, method_to_run=change_state)

def line_callback(data):
    global line

    # rospy.loginfo(rospy.get_caller_id() + "Line %s", data.data)
    line = data.data

def reset_gyro_angle():
    global reference_angle

    reference_angle = gyro_pos

def read_angle():
    return reference_angle - gyro_pos


def distance_callback(data):
    global left_distance, right_distance

    #rospy.loginfo(rospy.get_caller_id() + "Distances %s", data.data)
    left_distance = data.data[0]
    right_distance = data.data[1]
    # print("LD: ", left_distance)
    # print("RD: ", right_distance)


def gyro_callback(data):
    global gyro_pos

    #rospy.loginfo(rospy.get_caller_id() + "Gyro angle %s", str(data.data))
    gyro_pos = data.data


def encoders_callback(data):
    global encoder

    #rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

def pillar_callback(data):
    global pillar

    # rospy.loginfo(rospy.get_caller_id() + "Pillar %s", data.data)
    pillar = data
    # print("Pillar: ", pillar.data)

    
def dc_speed(speed=0):
    msg = str(speed)
    #rospy.loginfo(msg)
    pub.publish(msg)


def get_target(color, center, height, kc=1, kh=1, min_center=30):
        center = (100 + center)/2
        # print("Color:", color)
        # print("Center:", center)
        if color == 1 and center > min_center:
            t = kc * center + (kh * 100 * height)/300
            # print("t:", t)
            return min(max(-100, int(t)), 100)
        if color == -1 and center < 100 - min_center:
            t = kc * center + (kh * 100 * height)/300
            # print("t:", t)
            return -1 * min(max(-100, int(t)), 100)
        #     return -1 * min(max(-100, int(kc * (100 + center)/2 + (kh * 100 * height)/300)), 100)
        return None


def drive_gyro(speed=50, main_position=0, pid_ks=(1, 0, 0)):
    global line, current_path, previous_path, gyro_target

    # Initialise PID object
    pid = PID(pid_ks[0], pid_ks[1], pid_ks[2])
    pid.SetPoint = 0
    pid.setSampleTime(0.005)
    dc_speed(speed)
    print("RUNNING")
    
    turns = 0
    last_pillar_turn_ticks = 0
    pillar_ticks_delay = 600

    last_wall_turn_ticks = 0
    wall_ticks_delay = 350

    min_wall_distance = 80
    
    current_path = 0
    previous_path = -1
    turns = 0
    gyro_target = main_position
    direction = 0
    last_turn_ticks = 0
    turn_ticks = 9000
    while state == 1 and not rospy.is_shutdown() and turns < 666:
        # Check line to change gyro target
        error = read_angle() + gyro_target
        
        turn_left_distance = left_distance > 1000 and abs(error) < 20
        turn_right_distance = right_distance > 1000 and abs(error) < 20
        
        if turn_left_distance:
            direction = -1
        elif turn_right_distance:
            direction = 1
        if line != 0: # and ticks - last_line_turn_ticks > line_ticks_delay: #current_ms - last_line_turn_ms > line_delay:
            if encoder.data[1] - last_turn_ticks > turn_ticks:
                direction = line
            else:
                line = 0

        if encoder.data[1] - last_turn_ticks > turn_ticks and direction != 0:
            last_turn_ticks = encoder.data[1]
            gyro_target += -1*direction*90
            line = 0
            turns += 1
            rospy.loginfo("Turn " + str(turns) + ": " + str(direction) + ", ticks:" + str(encoder.data[1]))
        elif direction != 0:
            direction = 0
                    
        # 1st priority the wall
        if left_distance < min_wall_distance:
            rospy.loginfo("L-Wall fix")
            # rospy.loginfo("left wall " + str(left_distance))
            steer.set_steering(100)
            last_wall_turn_ticks = encoder.data[1]
            
        if right_distance < min_wall_distance:
            rospy.loginfo("R-Wall fix")
            # rospy.loginfo("right wall " + str(right_distance))
            steer.set_steering(-100)
            last_wall_turn_ticks = encoder.data[1]
            
        # 2nd priority the pillar
        # rospy.loginfo("pillar_cont")
        if encoder.data[1] - last_wall_turn_ticks > wall_ticks_delay:
            target = get_target(pillar.data[0], pillar.data[4], pillar.data[3], kc=0.6, kh=0.6, min_center=5)                
            if target is not None:
                rospy.loginfo("Pillar fix")
                # rospy.loginfo("pillar:" + str(target))
                steer.set_steering(target)
                last_pillar_turn_ticks = encoder.data[1]                
        if encoder.data[1] - last_pillar_turn_ticks > pillar_ticks_delay and encoder.data[1] - last_wall_turn_ticks > wall_ticks_delay:
            rospy.loginfo("Gyro fix")
            # Calculate error
            error = read_angle() + gyro_target
            # Calculate the correction value with the PID object
            pid.update(error)
            u = int(pid.output)
            current_steer = int(min(100, max(u, -100)))
            steer.set_steering(current_steer)
    rospy.loginfo("Ticks: " + str(encoder.data[1]))
    dc_speed(0)
    steer.set_steering(0)




def listener():
    rospy.init_node('run2', anonymous=False)

    rospy.Subscriber("line", Int8, line_callback)
    rospy.Subscriber("lrdistance", Int16MultiArray, distance_callback)
    rospy.Subscriber("gyro_angle", Int16, gyro_callback)
    rospy.Subscriber("pillar_topic", Int16MultiArray, pillar_callback)
    rospy.Subscriber("encoders", Int32MultiArray, encoders_callback)

    rate = rospy.Rate(100) 
    dc_speed(0)
    steer.set_steering(0)
    time.sleep(2)

    rospy.loginfo("Wait to push button")
    while state == 0:
        pass
    
    # while not rospy.is_shutdown():
    #     target = get_target(pillar.data[0], pillar.data[4], pillar.data[3], kc=0.6, kh=0.6, min_center=5)
    #     rospy.loginfo(str(target))
    #     rate.sleep()
    
    reset_gyro_angle()
    drive_gyro(speed=35, main_position=0, pid_ks=(1.5, 0.01, 0.002))
    
    
    GPIO.cleanup()

if __name__ == '__main__':
    listener()