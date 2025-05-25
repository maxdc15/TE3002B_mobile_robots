#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePose(Node):
    def __init__(self):
        super().__init__('turtle_pose_node')
        self.get_logger().info("Turtle pose estimation node initialized.")
        self.create_timer(0.001, self.real_pose_callback)
        self.sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 1)
        self.pub = self.create_publisher(Pose, 'turtle_real_pose', 1)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.msg = Pose()

    def pose_callback(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        self.lin_vel = msg.linear_velocity
        self.ang_vel = msg.angular_velocity

    def real_pose_callback(self):

        self.msg.x = self.x - 5.544444561004639
        self.msg.y = self.y - 5.544444561004639
        self.msg.theta = self.theta

        self.msg.linear_velocity = self.lin_vel
        self.msg.angular_velocity = self.ang_vel

        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    nodeh = TurtlePose()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()