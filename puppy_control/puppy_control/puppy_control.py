import rclpy
from rclpy.node import Node
import os, sys, math
import numpy as np
from std_msgs.msg import UInt8, UInt16, Float32, Float64, Bool, String,Float32MultiArray
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import Point32, Polygon
from puppy_interfaces.msg import Velocity, Pose, Gait
from puppy_interfaces.srv import SetRunActionName
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

sys.path.append('/home/ubuntu/PuppyPI_ws/src/puppy_control/driver')
from ServoCmd import setServoPulse, updatePulse
from ActionGroupControl import runActionGroup, stopActionGroup
from HiwonderPuppy import HiwonderPuppy, PWMServoParams


Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
#正常4腿站立
LieDown = {'roll':0.000, 'pitch':0.000, 'yaw':0.000, 'height':-5, 'x_shift':2, 'stance_x':0, 'stance_y':0}
#趴下
LookDown = {'roll':math.radians(0), 'pitch':math.radians(-15), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
LookDown_10deg = {'roll':math.radians(0), 'pitch':math.radians(-10), 'yaw':0.000, 'height':-9, 'x_shift':-0.1, 'stance_x':0, 'stance_y':0}
#俯身 10度
LookDown_20deg = {'roll':math.radians(0), 'pitch':math.radians(-20), 'yaw':0.000, 'height':-9, 'x_shift':-0.1, 'stance_x':0, 'stance_y':0}
#向下看
LookDown_30deg = {'roll':math.radians(0), 'pitch':math.radians(-30), 'yaw':0.000, 'height':-9.6, 'x_shift':-1.4, 'stance_x':1, 'stance_y':0}

StandLow = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-7, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
#4腿站立的低姿态
PuppyPose = LieDown.copy()
# stance_x：4条腿在x轴上额外分开的距离，单位cm
# stance_y：4条腿在y轴上额外分开的距离，单位cm
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm
# pitch： 狗身体的俯仰角，单位弧度


GaitConfigFast = {'overlap_time':0.1, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':5}
# 快速

GaitConfigSlow = {'overlap_time':0.4, 'swing_time':0.3, 'clearance_time':0.26, 'z_clearance':4}
# 慢速
GaitConfigMarkTime = {'overlap_time':0.2, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':5}
# GaitConfig = {'overlap_time':0.1, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':4}
# GaitConfigMarkTime = {'overlap_time':0.2, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':4}
# GaitConfigCrawl = {'overlap_time':0.4, 'swing_time':0.3, 'clearance_time':0.20, 'z_clearance':4}
GaitConfig = GaitConfigFast.copy()


# overlap_time:4脚全部着地的时间，单位秒
# swing_time：单脚离地时间，单位秒
# clearance_time：前后交叉脚相位间隔时间，单位秒
# z_clearance：走路时，脚尖要抬高的距离，单位cm


class Puppy(Node):
    def __init__(self):
        super().__init__('puppy_control')
        self.puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(),dof='8')

        self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        # puppy.gait_config(overlap_time = 0.1, swing_time = 0.15, z_clearance = 3)
        self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
        
        self.puppy.start() # 启动
        self.puppy.move_stop(servo_run_time = 500)#
        self.cmd_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 1)
        self.cmd_subscriber_  # prevent unused variable warning

    def stance(self, x = 0, y = 0, z = -11, x_shift = 2):# 单位cm
        # x_shift越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡
        return np.array([
                            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
                            [y, y, y, y],
                            [z, z, z, z],
                        ])#此array的组合方式不要去改变
    
    def cmd_callback(self, msg):
        global PuppyPose
        if abs(msg.linear.x) > 0.5 or abs(msg.angular.z) > 0.5:
            PuppyPose = Stand.copy()
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])

            if abs(msg.linear.x) > abs(msg.angular.z):
                self.MoveCmd(16*np.sign(msg.linear.x),0,0)
                print(16*np.sign(msg.linear.x))
            else:
                self.MoveCmd(0,0,np.radians(25)*np.sign(msg.angular.z))
        
        elif msg.linear.x == 0 and msg.angular.z == 0:
            self.MoveCmd(0,0,0)

        self.get_logger().info('I heard: "%s"' % msg)

    def MoveCmd(self, x, y, yaw_rate):
        if x == -999:# 原地踏步
            self.puppy.move(x=0, y=0, yaw_rate=0)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        elif x ==0 and y == 0 and yaw_rate == 0:
            self.puppy.move_stop(servo_run_time = 100)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        elif abs(x) <= 35 and abs(y) == 0 and abs(yaw_rate) <= np.radians(51):
            if x > 0:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']-0.8), PuppyPose['pitch'], PuppyPose['roll'])
            else:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']+0.8), PuppyPose['pitch'], PuppyPose['roll'])
            
            self.puppy.move(x=x, y=y, yaw_rate = yaw_rate)

def main(args=None):
    rclpy.init(args=args)

    PuppyControl = Puppy()

    rclpy.spin(PuppyControl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PuppyControl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()