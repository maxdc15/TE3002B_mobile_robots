#!/usr/bin/env python3

import rclpy, tf_transformations, math, time
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class Lyapunov(Node):
    def __init__(self):
        super().__init__('lyapunov_node')
        self.get_logger().info("Lyapunov controller node initialized.")

        self.pub_twist = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, '/arrived', 1)
        
        self.create_subscription(Odometry, "/odom", self.callback_odom, 1)
        self.create_subscription(Point, "/next_point", self.callback_points, 1)
        self.create_subscription(Bool, "/path_finished", self.callback_finished, 1)

        self.create_timer(0.01, self.lyapunov)
        
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
    
    # Code based on the progam given at "https://roboticoss.com/producto/control-de-posicion-con-orientacion-basado-en-lyapunov/"
    def lyapunov(self):
        if None in [self.x, self.y, self.theta, self.x_d, self.y_d]:
            return

        msg = Twist()

        # Lyapunov parameters
        K1 = 0.5
        K2 = 0.5
        q2 = 0.5

        # Calculate the distance to the target point and the angle
        Dx = self.x_d - self.x
        Dy = self.y_d - self.y
        l = math.sqrt(Dx**2 + Dy**2)
        theta_d = math.atan2(Dy, Dx)
        zeta = math.atan2(Dy, Dx) - self.theta
        psi = math.atan2(Dy, Dx) - theta_d

        # Normalize angles to [-pi, pi]
        zeta = math.atan2(math.sin(zeta), math.cos(zeta))
        psi  = math.atan2(math.sin(psi), math.cos(psi))

        # Control law
        v = K1 * math.cos(zeta) * l

        # Prevent singularities when zeta is close to zero
        if abs(zeta) > 1e-3:
            w = K2 * zeta + (K1 / zeta) * math.cos(zeta) * math.sin(zeta) * (zeta + q2 * psi)
        else:
            # aproximación cuando zeta ≈ 0
            w = K2 * zeta + K1 * q2 * psi

        # Publish the control command
        msg.linear.x = v
        msg.angular.z = w

        if l < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)
            self.get_logger().info("Arrived at the target point.")
        else:
            self.arrived.data = False
            self.pub_arrived.publish(self.arrived)
        self.pub_twist.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodeh = Lyapunov()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()