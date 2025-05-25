#!/usr/bin/env python3
import rclpy
from random import uniform
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class EncodersVelocityClass(Node):
    def __init__(self):
        super().__init__('encoders_velocity_node')
        self.get_logger().info("Robot enconders velocity node.")

        self.pub_wR = self.create_publisher(Float32, 'VelocityEncR', 1)
        self.pub_wL = self.create_publisher(Float32, 'VelocityEncL', 1)
        
        self.create_subscription(Twist, 'turtle1/cmd_vel', self.velocity_callback, 1)
        
        self.create_timer(0.01, self.encoders_vel_callback)

        self.v = 0.0
        self.w = 0.0
        
        self.r = 0.0505
        self.L = 0.1712
        self.noise = [-0.5, 0.4]

    def velocity_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def encoders_vel_callback(self):

        msg_wR = Float32()
        msg_wL = Float32()

        if self.v != 0 or self.w != 0:
            msg_wR.data = (self.v/self.r)+((self.w*self.L)/(2*self.r)) + uniform(-self.noise[0], self.noise[1])
            msg_wL.data = (self.v/self.r)-((self.w*self.L)/(2*self.r)) + uniform(-self.noise[0], self.noise[1])

        self.pub_wR.publish(msg_wR)
        self.pub_wL.publish(msg_wL)


def main(args=None):
    rclpy.init(args=args)
    nodeh = EncodersVelocityClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()