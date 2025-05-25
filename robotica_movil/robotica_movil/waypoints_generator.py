#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

# This node generates a path for the robot to follow by publishing target points.
class WaypointsGenerator(Node):
    def __init__ (self):
        super().__init__('waypoints_generator_node')
        self.get_logger().info("Waypoints generator node initialized.")
        self.get_logger().info("Generating path...")

        self.pub_point = self.create_publisher(Point, "/next_point", 1)
        self.pub_finished = self.create_publisher(Bool, "/path_finished", 1)
    
        self.create_subscription(Bool, "/arrived", self.callback_arrived, 10)

        self.create_timer(0.01, self.generate_path)     
        
        self.point_list =   [
                            #[3.0, -2.0]
                            #[4.0, -4.0]
                            [2.0, -3.0]
                            ]

        self.msg = Point()
        self.msg_finished = Bool()
        self.arrived = False
        self.msg_finished.data = False
        self.msg.x = 0.0
        self.msg.y = 0.0
        self.msg.z = 0.0
        self.t0 = time.time()
        
    # Callback function that is triggered when the robot reports it has arrived at a point
    def callback_arrived(self, msg):
        if msg.data:
            self.get_logger().info("Robot has arrived at the point x = %f, y = %f" % (self.msg.x, self.msg.y))
            self.arrived = True
            self.point_list.pop(0)
        else:
            self.arrived = False

    # Function to generate and publish the next point in the path
    def generate_path(self):
        if (len(self.point_list) > 0):
            [x, y] = self.point_list[0]
            self.msg.x = x
            self.msg.y = y

            self.pub_point.publish(self.msg) # Publish the current target point

        else:
            self.get_logger().info("Path generation finished")
            self.msg_finished.data = True
            self.pub_finished.publish(self.msg_finished)
            self.destroy_node()
            return

# Main function to initialize the ROS2 node and start the event loop
def main(args=None):
    rclpy.init(args=args)
    nodeh = WaypointsGenerator()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

# Main entry point for the script
if __name__ == "__main__":
    main()