import sys
import threading
from PWMServoControl import *
import time
from PuppyPI_IK import PuppyIK
from PuppyPI_GAIT import PuppyGait

import numpy as np
# import ActionGroupControl as AGC

class ControlCmd:
    def __init__(self):
        self.Servo = PWMServo()
        self.IK = PuppyIK()
        self.Gait = PuppyGait()

        # servo.setThreshold(1,1000,2300)
        self.Servo.setThreshold(2,1000,2300)
        # servo.setThreshold(3,1000,2300)
        self.Servo.setThreshold(4,1000,2300)
        # servo.setThreshold(5,1000,2300)
        self.Servo.setThreshold(6,1000,2300)
        # servo.setThreshold(7,1000,2300)
        self.Servo.setThreshold(8,1000,2300)

        self.t = 0
        self.dt = 0.05
        self.speed = 1

    def allServoRelease(self):
        self.Servo.unload(1)
        self.Servo.unload(2)
        self.Servo.unload(3)
        self.Servo.unload(4)
        self.Servo.unload(5)
        self.Servo.unload(6)
        self.Servo.unload(7)
        self.Servo.unload(8)


    def mapAngle2Pulse(self, angle):
        return int(angle*2000/180+500)


    def puppyMove(self, left_leg_move, right_leg_move, leg_height = 15):

        while self.t < 1 and (right_leg_move != 0 or left_leg_move != 0):   

            leg_pos = self.Gait.Trot(self.t, self.speed*25, leg_height, left_leg_move, left_leg_move, right_leg_move, right_leg_move)
            upper_leg_1, upper_leg_2, upper_leg_3, upper_leg_4, lower_leg_1, lower_leg_2, lower_leg_3, lower_leg_4 = self.IK.legInverseKinematics(leg_pos)

            leg_move_time = 25
            self.Servo.setPulse(3, self.mapAngle2Pulse(upper_leg_1), leg_move_time)
            self.Servo.setPulse(4, self.mapAngle2Pulse(lower_leg_1), leg_move_time)
            self.Servo.setPulse(1, self.mapAngle2Pulse(upper_leg_2), leg_move_time)
            self.Servo.setPulse(2, self.mapAngle2Pulse(lower_leg_2), leg_move_time)
            self.Servo.setPulse(5, self.mapAngle2Pulse(upper_leg_3), leg_move_time)
            self.Servo.setPulse(6, self.mapAngle2Pulse(lower_leg_3), leg_move_time)
            self.Servo.setPulse(7, self.mapAngle2Pulse(upper_leg_4), leg_move_time)
            self.Servo.setPulse(8, self.mapAngle2Pulse(lower_leg_4), leg_move_time)
            time.sleep(leg_move_time/1000)

            self.t += self.dt
        self.t = 0


if __name__ == "__main__":
    pass