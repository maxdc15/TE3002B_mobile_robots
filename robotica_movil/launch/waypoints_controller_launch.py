from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='0',
        description='Tipo de controlador: 0 = TurnWhileGo, 1 = TurnThenGo, 2 = PurePursuit, 3 = Lyapunov'
    )

    controller_type = LaunchConfiguration('controller_type')

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )

    odometry_node = Node(
        package='robotica_movil',
        executable='odometry',
        name='odometry',
    )

    encoders_velocity_node = Node(
        package='robotica_movil',
        executable='encoders_velocity',
        name='encoders_velocity',
    )
    
    robot_controller_node = Node(
        package='robotica_movil',
        executable='robot_controller',
        name='robot_controller',
        parameters=[{'controller_type': controller_type}],
    )

    waypoints_generator_node = Node(
        package='robotica_movil',
        executable='waypoints_generator',
        name='waypoints_generator',
    )

    l_d = LaunchDescription([turtlesim_node, odometry_node, encoders_velocity_node, controller_type_arg, robot_controller_node, waypoints_generator_node])
    
    return l_d