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

class PuppyGait():
    def __init__(self):
        self.h = PuppyPI_Config.puppy_height
    
    def Trot(self, t, x_target, z_target, r1, r4, r2, r3):
        Tf=0.5
        if t < Tf:
            phase_1_swing=self.swingCurveGenerate(t,Tf,x_target,z_target,0,0,0)
            phase_1_support=self.supportCurveGenerate(0.5+t,Tf,x_target,0.5,0)
            #TROT
            leg_1_x = phase_1_swing[0]*r1    ; leg_2_x = phase_1_support[0]*r2    ; leg_3_x = phase_1_swing[0]*r3    ; leg_4_x = phase_1_support[0]*r4
            leg_1_y = self.h-phase_1_swing[1]; leg_2_y = self.h-phase_1_support[1]; leg_3_y = self.h-phase_1_swing[1]; leg_4_y = self.h-phase_1_support[1]
            
        if t >= Tf:
            phase_2_swing=self.swingCurveGenerate(t-0.5,Tf,x_target,z_target,0,0,0)
            phase_2_support=self.supportCurveGenerate(t,Tf,x_target,0.5,0)
            #TROT
            leg_1_x = phase_2_support[0]*r1    ; leg_2_x = phase_2_swing[0]*r2    ; leg_3_x = phase_2_support[0]*r3    ; leg_4_x = phase_2_swing[0]*r4
            leg_1_y = self.h-phase_2_support[1]; leg_2_y = self.h-phase_2_swing[1]; leg_3_y = self.h-phase_2_support[1]; leg_4_y = self.h-phase_2_swing[1]

        return leg_1_x, leg_2_x, leg_3_x, leg_4_x, leg_1_y, leg_2_y, leg_3_y, leg_4_y
    
    def supportCurveGenerate(self, t, Tf, x_past, t_past, zf):
        # 当前时间；支撑相占空比；摆动相 x 最终位;t最终时间，支撑相 zf
        # Only X Generator
        average=x_past/(1-Tf)
        xf=x_past-average*(t-t_past)
        return xf,zf

    def swingCurveGenerate(self, t, Tf, xt, zh, x0, z0, xv0):
        #输入参数：当前时间；支撑相占空比；x方向目标位置，z方向抬腿高度，摆动相抬腿前腿速度
        # X Generator
        if t>=0 and t<Tf/4:
            xf=(-4*xv0/Tf)*t*t+xv0*t+x0
            
        if t>=Tf/4 and t<(3*Tf)/4:
            xf=((-4*Tf*xv0-16*xt+16*x0)*t*t*t)/(Tf*Tf*Tf)+((7*Tf*xv0+24*xt-24*x0)*t*t)/(Tf*Tf)+((-15*Tf*xv0-36*xt+36*x0)*t)/(4*Tf)+(9*Tf*xv0+16*xt)/(16)
            
        if t>(3*Tf)/4:
            xf=xt

        # Z Generator
        if t>=0 and t<Tf/2:
            zf=(16*z0-16*zh)*t*t*t/(Tf*Tf*Tf)+(12*zh-12*z0)*t*t/(Tf*Tf)+z0
        
        if t>=Tf/2:
            zf=(4*z0-4*zh)*t*t/(Tf*Tf)-(4*z0-4*zh)*t/(Tf)+z0
            
        #Record touch down position
        x_past=xf
        t_past=t
        
        # # Avoid zf to go zero
        if zf<=0:
            zf=0
        #x,z position,x_axis stop point,t_stop point;depend on when the leg stop
        
        return xf,zf,x_past,t_past
