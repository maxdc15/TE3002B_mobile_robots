#!/usr/bin/env python3

import rclpy, tf_transformations, math, time
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class TurnWhileGo(Node):
    def __init__(self):
        super().__init__('turn_while_go')
        self.get_logger().info("Turn while go controller node initialized.")

        self.pub_twist = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, '/arrived', 1)
        
        self.create_subscription(Odometry, "/odom", self.callback_odom, 1)
        self.create_subscription(Point, "/next_point", self.callback_points, 1)
        self.create_subscription(Bool, "/path_finished", self.callback_finished, 1)

        self.create_timer(0.01, self.turn_while_go)
        
        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.arrived = Bool()

        self.start_time = None

    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.theta = tf_transformations.euler_from_quaternion(quaternion)[2]
        
    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        self.get_logger().info(f"Next point: {self.x_d}, {self.y_d}")

        if self.start_time is None:
            self.start_time = time.time()

    def callback_finished(self, msg):
        if msg.data:
            self.get_logger().info("Path generation finished")
            self.x_d = None
            self.y_d = None
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)

            if self.start_time is not None:
                total_time = time.time() - self.start_time
                self.get_logger().info(f"Total time to complete the trajectory: {total_time:.2f} seconds")
            else:
                self.get_logger().warn("Unable to calculate total time, start time is None.")

            rclpy.shutdown()
            return

    def turn_while_go(self):
        if self.x is None or self.y is None or self.x_d is None or self.y_d is None:
            return

        msg = Twist()

        kv = 0.5
        kw = 1.0

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx ** 2 + Dy ** 2)
        
        if distance > 0.1:
            angle = math.atan2(Dy, Dx) - self.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            msg.linear.x = kv
            msg.angular.z = kw * angle
            self.arrived.data = False
            self.pub_arrived.publish(self.arrived)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)
        self.pub_twist.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodeh = TurnWhileGo()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()