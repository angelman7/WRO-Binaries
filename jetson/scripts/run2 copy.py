#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import String, Int16, Int16MultiArray, Int32MultiArray, Int8
import steering
from modules.pid import PID

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
# pillar_array.data = ["color", "cx", "cy", "height", "center"]
pillar = Int16MultiArray()
pillar.data = [0, 0, 0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
line = 0
do_turn = False

steer = steering.Steering()
reference_angle = 0

pub = rospy.Publisher('speed', String, queue_size=10)

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

def turn(angle):
    global do_turn

    print("start steering in demo turn function")
    time.sleep(0.5)
    print("stop steering in demo turn function")
    do_turn = False

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
    # Initialise PID object
    pid = PID(pid_ks[0], pid_ks[1], pid_ks[2])
    pid.SetPoint = 0
    pid.setSampleTime(0.005)
    dc_speed(speed)
    print("RUNNING")
    
    turns = 0
    last_turn_ms = 0

    turns = 0
    angle_target = main_position

    while True:
        current_ms = time.time() * 1000

        # print("Pillar.data", pillar.data)
        target = get_target(pillar.data[0], pillar.data[4], pillar.data[3], kc=0.6, kh=0.6, min_center=22)

        if target is not None:
            print("target:", target)
            steer.set_steering(target)
            time.sleep(1)
        else:
            current_ms = time.time() * 1000
            # Calculate error
            error = read_angle() + angle_target
            # Calculate the correction value with the PID object
            pid.update(error)
            u = int(pid.output)
            current_steer = int(min(100, max(u, -100)))
            steer.set_steering(current_steer)

        if line != 0  and current_ms - last_turn_ms > 5000:
            last_turn_ms = current_ms

            turns += 1
            angle_target += -1*line*90
            print("Line")
            
        # if current_ms - last_turn > 2000 and (left_distance > 700 or right_distance > 700):                    
        #     if turns == 0:
        #         start_distance = encoder.data[1] - starting_position[1]
        #         if left_distance > 700:
        #             direction = -1
        #         else:
        #             direction = 1
        #     last_turn = current_ms
        #     turns += 1
        #     pos += -1*direction*90


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
    time.sleep(5)

    reset_gyro_angle()
    drive_gyro(speed=40, main_position=0, pid_ks=(1.5, 0.1, 0.005))
    # while not rospy.is_shutdown():
    #     pass

if __name__ == '__main__':
    listener()