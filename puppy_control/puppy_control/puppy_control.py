#！usr/bin python3.8

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

sys.path.append('/home/puppypi/PuppyPI_ws/src/puppy_control/driver')
from ControlCmd import ControlCmd


class Puppy(Node):
    def __init__(self):
        super().__init__('puppy_control')
        self.controlCmd = ControlCmd()
        self.leg_height = 15
        self.cmd_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 1)
        self.cmd_subscriber_  # prevent unused variable warning
    
    def cmd_callback(self, msg):
        if abs(msg.linear.x) >= abs(msg.angular.z):

            if np.sign(msg.linear.x) > 0:
                left_leg_move  = 1
                right_leg_move = 1
            else:
                left_leg_move  = -1
                right_leg_move = -1
        else:

            if np.sign(msg.angular.z) > 0:
                left_leg_move  = -1
                right_leg_move = 1
            else:
                left_leg_move  = 1
                right_leg_move = -1

        self.controlCmd.puppyMove(left_leg_move, right_leg_move, self.leg_height)

        self.get_logger().info('I heard: "%s"' % msg)


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