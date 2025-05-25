#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class PathGenPoint(Node):
    def __init__(self):
        super().__init__('traj_gen_node')
        self.get_logger().info("Trajectory generator node initialized.")

        # Figure parameters
        self.declare_parameter('shape', 'triangle')  # 'square', 'triangle', 'circle'
        self.shape = self.get_parameter('shape').value

        # Publishers
        self.publisher_ = self.create_publisher(Point, 'next_point', 10)
        self.finish_pub = self.create_publisher(Bool, '/path_finished', 1)
        
        if self.shape == 'square':
            self.timer = self.create_timer(0.5, self.square_callback)
        elif self.shape == 'triangle':
            self.timer = self.create_timer(0.5, self.triangle_callback)
        elif self.shape == 'circle':
            self.timer = self.create_timer(0.5, self.circle_callback)
        else:
            self.get_logger().warn(f"Figura '{self.shape}' no reconocida. Se usará 'triangle' por defecto.")
            self.timer = self.create_timer(0.1, self.square_callback)

        self.msg = Point()

        self.t0 = time.time()
        self.finished = False


    def square_callback(self):
        elapsed_time = time.time() - self.t0
        robot_speed = 0.5
        t = elapsed_time * robot_speed
        #self.get_logger().info("Tiempo transcurrido: %f" % elapsed_time)
        #self.get_logger().info("Tiempo transcurrido: %f" % t)

        if 0 <= t <= 2:
            self.msg.x = t
            self.msg.y = 0.0
        elif 2 <= t <= 4:
            self.msg.x = 2.0
            self.msg.y = t - 2.0
        elif 4 <= t <= 6:
            self.msg.x = 6.0 - t
            self.msg.y = 2.0
        elif 6 <= t <= 8:
            self.msg.x = 0.0
            self.msg.y = 8.0 - t
        else:
            self.msg.x = 0.0
            self.msg.y = 0.0
            self.finished = True
            self.finish_pub.publish(Bool(data=True))

        self.publisher_.publish(self.msg)

    def triangle_callback(self):
        elapsed_time = time.time() - self.t0
        robot_speed = 0.5
        t = elapsed_time * robot_speed

        if 0 <= t <= 2:
            self.msg.x = t
            self.msg.y = 0.0
        elif 2 <= t <= 4:
            t_local = t - 2.0         
            self.msg.x = 2.0 - t_local * robot_speed
            self.msg.y = (math.sqrt(3) / 2) * t_local
        elif 4 <= t <= 6:
            t_local = t - 4.0   
            self.msg.x = 1.0 - t_local * robot_speed
            self.msg.y = math.sqrt(3) - (math.sqrt(3) / 2) * t_local
        else:
            self.msg.x = 0.0
            self.msg.y = 0.0
            self.finished = True
            self.finish_pub.publish(Bool(data=True))

        self.publisher_.publish(self.msg)

    def circle_callback(self):
        r = 2.0
        elapsed_time = time.time() - self.t0
        robot_speed = 0.25
        t = elapsed_time * robot_speed
        #self.get_logger().info("Tiempo transcurrido: %f" % elapsed_time)
        #self.get_logger().info("Tiempo transcurrido: %f" % t)
        if 0 <= t <= 6.5:
            self.msg.x = r * math.cos(t)-r
            self.msg.y = r * math.sin(t)
        else:
            self.msg.x = 0.0
            self.msg.y = 0.0
            self.finished = True
            self.finish_pub.publish(Bool(data=True))
        self.publisher_.publish(self.msg)
            
def main(args=None):
    rclpy.init(args=args)
    nodeh = PathGenPoint()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()