#project used to :
#          1. Interface motors (Lego or conventional) with microPython
#          2. Interface servo for steering(NOT TESTED)
#          3. Interface Lego color sensor(NOT READY)

#COMPONENTS:

#-----------------------L298N-----------------------------

#strip the black insulation off of the Lego wire with caution.
#you shoyld see 6 cables with different colors. strip the black and the white one
#connect them on L298N OUT1 and OUT2
#pins named ena, in1 and in2 have the same name on l298n
#ena : Pin ena is a PWM output connected to Pin 26 (to control speed)
#in1 : Pin in1 is a digital output (values 0/1) connected to 27 (to control direction)
#in2 : Pin in2 is a digital output (values 0/1) connected to 14 (to control direction)
#in1 and in2 values:
#    in1 | in2 | motor movement
#     0  |  0  |     STOP
#     0  |  1  |     CW
#     1  |  0  |     CCW
#     1  |  1  |     STOP
#speed value is -100 to 100 (negative for reverse)
#time value is in seconds

#----------------------SERVO------------------------------

                    #NOT TESTED
# black wire : GND
#   red wire : 5V
#yellow wire : SIGNAL 
#servo signal pin is connected to PWM Pin 25
#steer value is -1.0 to +1.0 (turn left or right)

#-------------------COLOR SENSOR--------------------------

                    #NOT READY


#METHODS:

# moveSpeed(speed) : turn motor with speed
# moveTime(speed, tme) : turn motor for time
# stop(): stop the main motor
# steering(steer) : steer the car using steer as input
# colorSensor() : get value for color sensor

#------------------------NOTES-----------------------------

#There will be many updates when added features are tested!!



from machine import Pin, PWM
from time import sleep, time

class Actuators:
    def __init__(self):
        
        self.ena = PWM(Pin(26), freq = 1000, duty = 0) #main motor PWM PIN
        self.in1 = Pin(27,Pin.OUT) #main motor 
        self.in2 = Pin(14,Pin.OUT) #main motor
        self.servo = PWM(Pin(25), freq = 1000, duty = 0) #servo steering
        
    def moveSpeed(self, speed):
        
        if (speed > 0 and speed <= 100):
            motorSpeed = speed*10
            self.in1.value(1)
            self.in2.value(0)
            
        elif(speed < 0 and speed >= -100):
            motorSpeed = speed*(-10)
            self.in1.value(0)
            self.in2.value(1)
            
        self.ena.duty(motorSpeed)
        
    def moveTime(self, speed, tme):
        
        
        if (speed > 0 and speed <= 100):
            self.in1.value(1)
            self.in2.value(0)
            motorSpeed = speed*10
            
        elif (speed < 0 and speed >= -100):
            self.in1.value(0)
            self.in2.value(1)
            motorSpeed = speed*(-10)

        self.ena.duty(motorSpeed)
        sleep(tme)
        self.in1.value(0)
        self.in2.value(0)
    
    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        
    def steering(self, steer):
        steerValue = steer*1000
        self.servo.duty(steerValue)
        
    def colorSensor(self):
        print("color sensor")#debug
        
move = Actuators()
#move.moveSpeed(speed)
#move.moveTime(speed, tme) 
#move.steering(steer)
#move.colorSensor()
move.stop()
