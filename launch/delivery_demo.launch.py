import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory('restaurant_delivery')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_dir, 'worlds', 'restaurant.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'delivery_robot.urdf')

    # Read URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    target_x = LaunchConfiguration('target_x', default='4.0')
    target_y = LaunchConfiguration('target_y', default='0.0')
    stop_distance = LaunchConfiguration('stop_distance', default='0.45')
    child_delay = LaunchConfiguration('child_delay', default='3.5')
    debug_mode = LaunchConfiguration('debug_mode', default='true')

    # 1. Gazebo Server & Client (Include gazebo_ros empty_world with restaurant.world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Spawn Robot in Gazebo beside kitchen counter (X=-1.2, Y=0.0)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'delivery_robot', '-file', urdf_path, '-x', '-1.2', '-y', '0.0', '-z', '0.05'],
        output='screen'
    )

    # 4. Obstacle Monitor Node
    obstacle_monitor_node = Node(
        package='restaurant_delivery',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'stop_distance': stop_distance,
            'warning_distance': 0.85,
            'front_angle_deg': 30.0,
            'clear_consecutive_count': 4,
            'use_sim_time': use_sim_time
        }]
    )

    # 5. Delivery Controller Node
    delivery_controller_node = Node(
        package='restaurant_delivery',
        executable='delivery_controller',
        name='delivery_controller',
        output='screen',
        parameters=[{
            'target_x': target_x,
            'target_y': target_y,
            'cruise_speed': 0.25,
            'avoid_angular_speed': 0.4,
            'goal_tolerance': 0.20,
            'use_sim_time': use_sim_time
        }]
    )

    # 6. Child Spawner (Dynamic intrusion node triggered by robot position)
    child_spawner_node = Node(
        package='restaurant_delivery',
        executable='child_spawner',
        name='child_spawner',
        output='screen',
        parameters=[{
            'entity_name': 'running_child',
            'trigger_robot_x': 1.0,
            'start_delay_sec': child_delay,
            'start_x': 2.0,
            'start_y': 2.5,
            'target_y': -1.5,
            'run_speed': 1.5,
            'use_sim_time': use_sim_time
        }]
    )

    # 7. Debug Recorder Node (Captures frames at 1Hz, generates GIF and Contact Sheet)
    debug_recorder_node = Node(
        package='restaurant_delivery',
        executable='debug_recorder',
        name='debug_recorder',
        output='screen',
        condition=IfCondition(debug_mode),
        parameters=[{
            'record_fps': 4.0,
            'gif_duration_ms': 250,
            'output_dir': '/home/ubuntu/pai_ws/src/restaurant_delivery/debug_output',
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('target_x', default_value='4.0', description='Target table X coordinate'),
        DeclareLaunchArgument('target_y', default_value='0.0', description='Target table Y coordinate'),
        DeclareLaunchArgument('stop_distance', default_value='0.45', description='Emergency stop threshold distance'),
        DeclareLaunchArgument('child_delay', default_value='3.5', description='Seconds before child starts running'),
        DeclareLaunchArgument('debug_mode', default_value='true', description='Enable 1Hz frame debug recorder & GIF export'),

        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        obstacle_monitor_node,
        delivery_controller_node,
        child_spawner_node,
        debug_recorder_node
    ])
