from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    aiil_rosbot_demo_dir = get_package_share_directory('aiil_rosbot_demo')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    wall_distance = LaunchConfiguration('wall_distance')
    max_speed = LaunchConfiguration('max_speed')
    min_speed = LaunchConfiguration('min_speed')
    max_rotation = LaunchConfiguration('max_rotation')
    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')
    scan_angle = LaunchConfiguration('scan_angle')
    marker_spin_velocity = LaunchConfiguration('marker_spin_velocity')
    left_wall_following = LaunchConfiguration('left_wall_following')
    min_front_dist = LaunchConfiguration('min_front_dist')
    hazard_scan_interval = LaunchConfiguration('hazard_scan_interval')
    corner_detection_angle = LaunchConfiguration('corner_detection_angle')
    explore_timeout = LaunchConfiguration('explore_timeout')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_wall_distance_cmd = DeclareLaunchArgument(
        'wall_distance',
        default_value='0.5',
        description='Target distance from the wall for wall following'
    )
    
    declare_max_speed_cmd = DeclareLaunchArgument(
        'max_speed',
        default_value='0.3',
        description='Maximum forward speed'
    )
    
    declare_min_speed_cmd = DeclareLaunchArgument(
        'min_speed',
        default_value='0.1',
        description='Minimum forward speed'
    )
    
    declare_max_rotation_cmd = DeclareLaunchArgument(
        'max_rotation',
        default_value='1.0',
        description='Maximum rotation speed'
    )
    
    declare_kp_cmd = DeclareLaunchArgument(
        'kp',
        default_value='2.0',
        description='Proportional gain for PID controller'
    )
    
    declare_ki_cmd = DeclareLaunchArgument(
        'ki',
        default_value='0.0',
        description='Integral gain for PID controller'
    )
    
    declare_kd_cmd = DeclareLaunchArgument(
        'kd',
        default_value='0.5',
        description='Derivative gain for PID controller'
    )
    
    declare_scan_angle_cmd = DeclareLaunchArgument(
        'scan_angle',
        default_value='70.0',
        description='Angle range to consider for wall following (degrees)'
    )
    
    declare_marker_spin_velocity_cmd = DeclareLaunchArgument(
        'marker_spin_velocity',
        default_value='0.5',
        description='Angular velocity when spinning to detect markers'
    )
    
    declare_left_wall_following_cmd = DeclareLaunchArgument(
        'left_wall_following',
        default_value='true',
        description='True for left wall following, False for right'
    )
    
    declare_min_front_dist_cmd = DeclareLaunchArgument(
        'min_front_dist',
        default_value='0.5',
        description='Minimum front distance for collision avoidance'
    )
    
    declare_hazard_scan_interval_cmd = DeclareLaunchArgument(
        'hazard_scan_interval',
        default_value='20.0',
        description='Time between hazard scans in seconds'
    )
    
    declare_corner_detection_angle_cmd = DeclareLaunchArgument(
        'corner_detection_angle',
        default_value='45.0',
        description='Angle to detect corners (degrees)'
    )
    
    declare_explore_timeout_cmd = DeclareLaunchArgument(
        'explore_timeout',
        default_value='240.0',
        description='Maximum exploration time in seconds (4 minutes)'
    )
    
    # Launch the wall following node
    wall_following_node = Node(
        package='aiil_rosbot_demo',
        executable='nav_wall_follow',
        name='nav_wall_follow',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'wall_distance': wall_distance},
            {'max_speed': max_speed},
            {'min_speed': min_speed},
            {'max_rotation': max_rotation},
            {'kp': kp},
            {'ki': ki},
            {'kd': kd},
            {'scan_angle': scan_angle},
            {'marker_spin_velocity': marker_spin_velocity},
            {'left_wall_following': left_wall_following},
            {'min_front_dist': min_front_dist},
            {'hazard_scan_interval': hazard_scan_interval},
            {'corner_detection_angle': corner_detection_angle},
            {'explore_timeout': explore_timeout},
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_wall_distance_cmd)
    ld.add_action(declare_max_speed_cmd)
    ld.add_action(declare_min_speed_cmd)
    ld.add_action(declare_max_rotation_cmd)
    ld.add_action(declare_kp_cmd)
    ld.add_action(declare_ki_cmd)
    ld.add_action(declare_kd_cmd)
    ld.add_action(declare_scan_angle_cmd)
    ld.add_action(declare_marker_spin_velocity_cmd)
    ld.add_action(declare_left_wall_following_cmd)
    ld.add_action(declare_min_front_dist_cmd)
    ld.add_action(declare_hazard_scan_interval_cmd)
    ld.add_action(declare_corner_detection_angle_cmd)
    ld.add_action(declare_explore_timeout_cmd)
    
    # Add wall following node
    ld.add_action(wall_following_node)
    
    return ld