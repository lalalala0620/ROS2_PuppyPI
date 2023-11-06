import sys
import threading
from PWMServoControl import *
import time
import PuppyPI_GAITandIK
import numpy as np
# import ActionGroupControl as AGC

servo = PWMServo()
puppy = PuppyPI_GAITandIK.Puppy_Control()

# servo.setThreshold(1,1000,2300)
servo.setThreshold(2,1000,2300)
# servo.setThreshold(3,1000,2300)
servo.setThreshold(4,1000,2300)
# servo.setThreshold(5,1000,2300)
servo.setThreshold(6,1000,2300)
# servo.setThreshold(7,1000,2300)
servo.setThreshold(8,1000,2300)

def getServoPulse(id):
    return 0#getBusServoPulse(id)

def getServoDeviation(id):
    return servo.getDeviation(id)

def setServoPulse(id, pulse, use_time):
    servo.setPulse(id, pulse, use_time)

def setServoDeviation(id ,dev):
    servo.setDeviation(id, dev)
    
def saveServoDeviation(id):
    servo.saveDeviation(id)

def unloadServo(id):
    servo.unload(id)

def updatePulse(id):
    servo.updatePulse(id)

def allServoRelease():
    servo.unload(1)
    servo.unload(2)
    servo.unload(3)
    servo.unload(4)
    servo.unload(5)
    servo.unload(6)
    servo.unload(7)
    servo.unload(8)


def mapAngle2Pulse(angle):
    return int(angle*2000/180+500)

def puppyMove(left_leg_move, right_leg_move, leg_height = 15):

    leg_move_cycle = 1
    while leg_move_cycle <= 20:   

        
        upper_leg_1, upper_leg_2, upper_leg_3, upper_leg_4, lower_leg_1, lower_leg_2, lower_leg_3, lower_leg_4 = puppy.runTrot(left_leg_move, right_leg_move, leg_height)

        leg_move_time = 25
        setServoPulse(3, mapAngle2Pulse(upper_leg_1), leg_move_time)
        setServoPulse(4, mapAngle2Pulse(lower_leg_1), leg_move_time)
        setServoPulse(1, mapAngle2Pulse(upper_leg_2), leg_move_time)
        setServoPulse(2, mapAngle2Pulse(lower_leg_2), leg_move_time)
        setServoPulse(5, mapAngle2Pulse(upper_leg_3), leg_move_time)
        setServoPulse(6, mapAngle2Pulse(lower_leg_3), leg_move_time)
        setServoPulse(7, mapAngle2Pulse(upper_leg_4), leg_move_time)
        setServoPulse(8, mapAngle2Pulse(lower_leg_4), leg_move_time)
        time.sleep(leg_move_time/1000)

        leg_move_cycle += 1


if __name__ == "__main__":
    pass