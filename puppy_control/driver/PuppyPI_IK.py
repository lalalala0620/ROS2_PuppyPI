from math import asin,acos,atan,pi,sqrt
import PuppyPI_Config

# Servo: 
# 
#  (o o)
# 1-----2
#   |||
#   |||
# 4-----3
#    *
#    *

class PuppyIK():
    def __init__(self):
        self.upper_leg_length = PuppyPI_Config.upper_leg_length
        self.lower_leg_length = PuppyPI_Config.lower_leg_length

    def legInverseKinematics(self, leg_pos):
        leg_1_x, leg_2_x, leg_3_x, leg_4_x, leg_1_y, leg_2_y, leg_3_y, leg_4_y = leg_pos[0], leg_pos[1], leg_pos[2], leg_pos[3], leg_pos[4], leg_pos[5], leg_pos[6], leg_pos[7]
        #Leg1
        leg_1_x = -leg_1_x
        lower_leg_1 = pi-acos((leg_1_x**2+leg_1_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_1 = acos((self.upper_leg_length**2+leg_1_x**2+leg_1_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_1_x**2+leg_1_y**2)))
        
        if leg_1_x>0:
            upper_leg_1 = (abs(atan(leg_1_y/leg_1_x))-phi_1)
        elif leg_1_x < 0:
            upper_leg_1 = (pi-abs(atan(leg_1_y/leg_1_x))-phi_1)
        else:
            upper_leg_1 = (pi-1.5707-phi_1)

        lower_leg_1=180 * lower_leg_1 / pi
        upper_leg_1=180 - 180 * upper_leg_1 / pi

        #Leg2
        leg_2_x = -leg_2_x
        lower_leg_2 = pi-acos((leg_2_x**2+leg_2_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_2 = acos((self.upper_leg_length**2+leg_2_x**2+leg_2_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_2_x**2+leg_2_y**2)))
        
        if leg_2_x>0:
            upper_leg_2=abs(atan(leg_2_y/leg_2_x))-phi_2
        elif leg_2_x<0:
            upper_leg_2=pi-abs(atan(leg_2_y/leg_2_x))-phi_2
        else:
            upper_leg_2=pi-1.5707-phi_2

        lower_leg_2=180-180*lower_leg_2/pi
        upper_leg_2=180*upper_leg_2/pi

        #Leg3
        leg_3_x = -leg_3_x
        lower_leg_3 = pi-acos((leg_3_x**2+leg_3_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_3 = acos((self.upper_leg_length**2+leg_3_x**2+leg_3_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_3_x**2+leg_3_y**2)))
        
        if leg_3_x > 0:
            upper_leg_3 = abs(atan(leg_3_y/leg_3_x))-phi_3
        elif leg_3_x < 0:
            upper_leg_3 = pi-abs(atan(leg_3_y/leg_3_x))-phi_3
        else:
            upper_leg_3 = pi-1.5707-phi_3

        lower_leg_3 = 180-180*lower_leg_3/pi
        upper_leg_3 = 180*upper_leg_3/pi

        #Leg4
        leg_4_x = -leg_4_x
        lower_leg_4 = pi-acos((leg_4_x**2+leg_4_y**2-self.upper_leg_length**2-self.lower_leg_length**2)/(-2*self.upper_leg_length**2))
        phi_4 = acos((self.upper_leg_length**2+leg_4_x**2+leg_4_y**2-self.lower_leg_length**2)/(2*self.upper_leg_length*sqrt(leg_4_x**2+leg_4_y**2)))
        
        if leg_4_x > 0:
            upper_leg_4 = abs(atan(leg_4_y/leg_4_x))-phi_4
        elif leg_4_x < 0:
            upper_leg_4 = pi-abs(atan(leg_4_y/leg_4_x))-phi_4
        else:
            upper_leg_4 = pi-1.5707-phi_4

        lower_leg_4 = 180*lower_leg_4/pi
        upper_leg_4 = 180-180*upper_leg_4/pi
        
        return upper_leg_1, upper_leg_2, upper_leg_3, upper_leg_4, lower_leg_1, lower_leg_2, lower_leg_3, lower_leg_4
    