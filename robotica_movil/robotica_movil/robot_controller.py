#!/usr/bin/env python3
import rclpy, tf_transformations, math, time
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.get_logger().info("Robot controller node initialized.")

        self.pub_twist = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.pub_arrived = self.create_publisher(Bool, '/arrived', 1)
        
        self.create_subscription(Odometry, '/odom', self.callback_odom, 1)
        self.create_subscription(Point, '/next_point', self.callback_points, 1)
        self.create_subscription(Pose, 'turtle_real_pose', self.callback_real_pose, 1)
        self.create_subscription(Bool, '/path_finished', self.callback_finished, 1)

        self.create_timer(0.01, self.control_callback)

        self.declare_parameter('controller_type', 2) # 0: turn_while_go, 1: turn_then_go, 2: pure_pursuit, 3: lyapunov
        self.controller_type = self.get_parameter('controller_type').value
        
        self.x = None
        self.y = None
        self.theta = None

        self.x_d = None
        self.y_d = None

        self.x_real = None
        self.y_real = None

        self.arrived = Bool()

        self.start_time = None
        self.itae = 0.0
        self.error_log_time = time.time()

    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.theta = tf_transformations.euler_from_quaternion(quaternion)[2]

    def callback_points(self, msg):
        self.x_d = msg.x
        self.y_d = msg.y

        if self.start_time is None:
            self.start_time = time.time()
            self.error_log_time = self.start_time
    
    def callback_real_pose(self, msg):
        self.x_real = msg.x
        self.y_real = msg.y

    def callback_finished(self, msg):
        if msg.data:
            self.get_logger().info("Path generation finished")
            self.x_d = None
            self.y_d = None
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)

            if self.start_time is not None:
                total_time = time.time() - self.start_time
                self.get_logger().info(f"Tiempo total: {total_time:.2f} segundos")
                self.get_logger().info(f"ITAE (∫ t·|e(t)| dt): {self.itae:.4f}")
            else:
                self.get_logger().warn("No se pudo calcular el tiempo total: start_time es None.")

            rclpy.shutdown()

    def compute_itae(self):
        if any(v is None for v in [self.x_real, self.y_real, self.x_d, self.y_d]):
            return

        current_time = time.time()
        
        if not hasattr(self, 'error_log_time'):
            self.error_log_time = current_time
            return

        elapsed = current_time - self.error_log_time
        self.error_log_time = current_time

        error = math.sqrt((self.x_real - self.x_d)**2 + (self.y_real - self.y_d)**2)
        self.itae += elapsed * error
        #self.get_logger().info(f"Error actual: {error:.4f} - ITAE acumulado: {self.itae:.4f}")
        
    def turn_while_go(self):
        if self.x is None or self.y is None or self.x_d is None or self.y_d is None:
            return

        msg = Twist()

        kv = 0.5
        kw = 2.0

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

    def turn_then_go(self):
        if self.x is None or self.y is None or self.x_d is None or self.y_d is None:
            return
        
        msg = Twist()

        kv = 0.5
        kw = 2.0

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx ** 2 + Dy ** 2)

        angle= math.atan2(Dy, Dx) - self.theta
        angle = math.atan2(math.sin(angle), math.cos(angle))

        if distance > 0.1:
            if abs(angle) > 0.08:  # Fase de rotación
                msg.linear.x = 0.0
                msg.angular.z = kw * angle
            else:  # Fase de avance
                msg.linear.x = kv * distance
                msg.angular.z = 0.0
                self.arrived.data = False
                self.pub_arrived.publish(self.arrived)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)
        self.pub_twist.publish(msg)

    def pure_pursuit(self):
        if self.x is None or self.y is None or self.x_d is None or self.y_d is None:
            return

        msg = Twist()

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx**2 + Dy**2)

        # Lookahead
        L = 0.8

        if distance > 0.1:
            cq = math.cos(self.theta)
            sq = math.sin(self.theta)

            msg.linear.x = Dx * cq + Dy * sq
            msg.angular.z = (1.0 / L) * (Dy*cq - Dx*sq)

            self.arrived.data = False
            self.pub_arrived.publish(self.arrived)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.arrived.data = True
            self.pub_arrived.publish(self.arrived)
            self.get_logger().info("Arrived at the target point.")

        self.pub_twist.publish(msg)

    # Code based on the progam given at "https://roboticoss.com/producto/control-de-posicion-con-orientacion-basado-en-lyapunov/"
    def lyapunov(self):
        if None in [self.x, self.y, self.theta, self.x_d, self.y_d]:
            return

        msg = Twist()

        # Lyapunov parameters
        K1 = 0.5
        K2 = 2.0
        q2 = 2.0

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

    def control_callback(self):
        if self.controller_type == 0:
            self.turn_while_go()
        elif self.controller_type == 1:
            self.turn_then_go()
        elif self.controller_type == 2:
            self.pure_pursuit()
        elif self.controller_type == 3:
            self.lyapunov()
        else:
            self.get_logger().error("Invalid controller type. Please choose 0, 1, 2, or 3.")
        
        if None not in [self.x_real, self.y_real, self.x_d, self.y_d]:
            self.compute_itae()


def main(args=None):
    rclpy.init(args=args)
    nodeh = RobotController()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()