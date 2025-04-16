#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import math
import numpy as np
from enum import Enum, auto
from collections import deque
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Empty
from nav2_msgs.action import NavigateToPose, Spin
from action_msgs.msg import GoalStatus


class RobotState(Enum):
    IDLE = auto()
    WAITING_FOR_START = auto()
    EXPLORING = auto()
    SCANNING_FOR_HAZARDS = auto()
    RETURNING_HOME = auto()
    STOPPED = auto()


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('nav_wall_follow')
        
        # Define Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wall_distance', 0.5),     # Target distance from wall
                ('max_speed', 0.3),         # Maximum forward speed
                ('min_speed', 0.1),         # Minimum forward speed
                ('max_rotation', 1.0),      # Maximum rotation speed
                ('kp', 2.0),                # Proportional gain
                ('ki', 0.0),                # Integral gain
                ('kd', 0.5),                # Derivative gain
                ('scan_angle', 70.0),       # Angle range to consider for wall following (degrees)
                ('marker_spin_velocity', 0.5),  # Angular velocity when spinning to detect markers
                ('left_wall_following', True),  # True for left wall following, False for right
                ('min_front_dist', 0.5),    # Minimum front distance for collision avoidance
                ('hazard_scan_interval', 20.0), # Time between hazard scans in seconds
                ('corner_detection_angle', 45.0), # Angle to detect corners (degrees)
                ('explore_timeout', 240.0),  # Maximum exploration time in seconds (4 minutes)
            ]
        )
        
        # Get parameter values
        self.wall_distance = self.get_parameter('wall_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_rotation = self.get_parameter('max_rotation').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.scan_angle = self.get_parameter('scan_angle').value
        self.marker_spin_velocity = self.get_parameter('marker_spin_velocity').value
        self.left_wall_following = self.get_parameter('left_wall_following').value
        self.min_front_dist = self.get_parameter('min_front_dist').value
        self.hazard_scan_interval = self.get_parameter('hazard_scan_interval').value
        self.corner_detection_angle = self.get_parameter('corner_detection_angle').value
        self.explore_timeout = self.get_parameter('explore_timeout').value
        
        # PID controller state
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Robot state
        self.state = RobotState.WAITING_FOR_START
        self.start_time = None
        self.last_hazard_scan_time = None
        self.home_position = None
        
        # Path tracking
        self.explore_path = []
        self.return_path = []
        
        # Hazard markers found
        self.hazard_markers = {}
        
        # Setup TF listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.explore_path_pub = self.create_publisher(Path, '/path_explore', 10)
        self.return_path_pub = self.create_publisher(Path, '/path_return', 10)
        self.hazards_pub = self.create_publisher(Marker, '/hazards', 10)
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan',
            self.scan_callback, 
            sensor_qos
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Subscribe to trigger topics
        self.start_sub = self.create_subscription(
            Empty,
            '/trigger_start',
            self.trigger_start_callback,
            10
        )
        
        self.teleop_sub = self.create_subscription(
            Empty,
            '/trigger_teleop',
            self.trigger_teleop_callback,
            10
        )
        
        self.return_home_sub = self.create_subscription(
            Empty,
            '/trigger_home',
            self.trigger_return_home_callback,
            10
        )
        
        # Nav2 action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')
        
        # Main control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Scanner data
        self.last_scan = None
        
        # Initialization complete
        self.get_logger().info('Wall-following navigation node initialized')
        self.publish_status("Waiting for start signal")
        
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.last_scan = msg
        
    def map_callback(self, msg):
        """Process map updates"""
        # Could be used for frontier detection or path planning
        pass
    
    def trigger_start_callback(self, msg):
        """Handle the start trigger"""
        if self.state == RobotState.WAITING_FOR_START:
            self.get_logger().info('Start signal received')
            self.state = RobotState.EXPLORING
            self.start_time = self.get_clock().now()
            self.last_hazard_scan_time = self.start_time
            
            # Record home position
            self.home_position = self.get_current_position()
            if self.home_position:
                self.get_logger().info(f'Home position set to: {self.home_position.position.x}, {self.home_position.position.y}')
                # Add the starting point to the exploration path
                self.add_to_explore_path(self.home_position)
            else:
                self.get_logger().warn('Could not determine home position')
    
    def trigger_teleop_callback(self, msg):
        """Handle the teleop trigger"""
        self.get_logger().info('Teleop mode triggered - stopping autonomous navigation')
        self.stop_robot()
        self.publish_status("Teleop mode active")
    
    def trigger_return_home_callback(self, msg):
        """Handle the return home trigger"""
        self.get_logger().info('Return home triggered')
        self.state = RobotState.RETURNING_HOME
        self.publish_status("Returning home")
    
    def get_current_position(self):
        """Get current robot position in map frame"""
        try:
            # Get the transform from base_link to map
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            
            # Create a pose from the transform
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Only log at debug level to avoid flooding the console
            self.get_logger().debug(f'Failed to get current position: {e}')
            return None
        
    def add_to_explore_path(self, pose):
        """Add current position to exploration path"""
        # Create a PoseStamped and add to path
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        self.explore_path.append(pose_stamped)
        
        # Publish the updated path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.explore_path
        
        self.explore_path_pub.publish(path_msg)
    
    def publish_return_path(self):
        """Publish the return path"""
        # Create the return path (reversed explore path)
        self.return_path = list(reversed(self.explore_path))
        
        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.return_path
        
        self.return_path_pub.publish(path_msg)
    
    def publish_status(self, status):
        """Publish robot status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def publish_hazard_marker(self, marker_id, position):
        """Publish a hazard marker"""
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.id = marker_id
        
        marker_msg.pose.position.x = position.x
        marker_msg.pose.position.y = position.y
        marker_msg.pose.position.z = position.z
        marker_msg.pose.orientation.w = 1.0
        
        # Set marker size and color
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        
        # Set color based on hazard ID
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (1.0, 0.5, 0.0),  # Orange
            (1.0, 1.0, 0.0),  # Yellow
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (0.5, 0.0, 1.0),  # Purple
            (1.0, 0.0, 1.0),  # Magenta
            (0.5, 0.5, 0.5),  # Gray
            (1.0, 1.0, 1.0),  # White
            (0.0, 0.0, 0.0),  # Black
            (0.8, 0.2, 0.2),  # Dark Red
            (0.2, 0.8, 0.2),  # Dark Green
            (0.2, 0.2, 0.8),  # Dark Blue
        ]
        
        color_idx = marker_id % len(colors)
        marker_msg.color.r = colors[color_idx][0]
        marker_msg.color.g = colors[color_idx][1]
        marker_msg.color.b = colors[color_idx][2]
        marker_msg.color.a = 1.0
        
        # Infinite lifetime
        marker_msg.lifetime.sec = 0
        
        self.hazards_pub.publish(marker_msg)
        self.get_logger().info(f'Published hazard marker {marker_id} at position ({position.x}, {position.y})')
    
    def start_spin_action(self):
        """Start a 360-degree spin to scan for hazards"""
        self.get_logger().info('Starting spin action to scan for hazards')
        
        # Create spin goal
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = 2 * math.pi  # 360 degrees in radians
        
        # Wait for action server
        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Spin action server not available')
            return False
        
        # Send goal
        self._spin_send_goal_future = self.spin_client.send_goal_async(
            spin_goal,
            feedback_callback=self.spin_feedback_callback
        )
        
        self._spin_send_goal_future.add_done_callback(self.spin_goal_response_callback)
        return True
    
    def spin_goal_response_callback(self, future):
        """Handle the response from the spin action server"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Spin goal rejected')
            self.state = RobotState.EXPLORING  # Resume exploration
            return
        
        self.get_logger().info('Spin goal accepted')
        
        self._spin_get_result_future = goal_handle.get_result_async()
        self._spin_get_result_future.add_done_callback(self.spin_result_callback)
    
    def spin_feedback_callback(self, feedback_msg):
        """Process feedback from the spin action"""
        pass  # We don't need to do anything with the feedback
        
    def spin_result_callback(self, future):
        """Handle the result of the spin action"""
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Spin completed successfully')
        else:
            self.get_logger().warn(f'Spin failed with status: {status}')
        
        # Resume exploration
        self.state = RobotState.EXPLORING
    
    def process_laser_scan(self):
        """Process laser scan data for wall following"""
        if self.last_scan is None:
            self.get_logger().warn('No laser scan data available')
            return None, None, None
        
        scan = self.last_scan
        
        # Get the range of indices that correspond to our scan angle
        angle_range = math.radians(self.scan_angle)
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        
        # Calculate front, side, and diagonal distances
        num_readings = len(scan.ranges)
        front_idx = int(num_readings / 2)
        
        # Define the ranges for different sections
        if self.left_wall_following:
            # Left wall following
            side_min_idx = int((math.pi / 2 - angle_range / 2) / angle_increment)
            side_max_idx = int((math.pi / 2 + angle_range / 2) / angle_increment)
            
            front_min_idx = int((-angle_range / 2) / angle_increment) + front_idx
            front_max_idx = int((angle_range / 2) / angle_increment) + front_idx
            
            corner_min_idx = int((math.pi / 2 - math.radians(self.corner_detection_angle)) / angle_increment)
            corner_max_idx = int((math.pi / 2) / angle_increment)
        else:
            # Right wall following
            side_min_idx = int((-math.pi / 2 - angle_range / 2) / angle_increment) + num_readings
            side_max_idx = int((-math.pi / 2 + angle_range / 2) / angle_increment) + num_readings
            
            front_min_idx = int((-angle_range / 2) / angle_increment) + front_idx
            front_max_idx = int((angle_range / 2) / angle_increment) + front_idx
            
            corner_min_idx = int((-math.pi / 2) / angle_increment) + num_readings
            corner_max_idx = int((-math.pi / 2 + math.radians(self.corner_detection_angle)) / angle_increment) + num_readings
        
        # Ensure indices are within the valid range
        side_min_idx = max(0, min(side_min_idx, num_readings - 1))
        side_max_idx = max(0, min(side_max_idx, num_readings - 1))
        front_min_idx = max(0, min(front_min_idx, num_readings - 1))
        front_max_idx = max(0, min(front_max_idx, num_readings - 1))
        corner_min_idx = max(0, min(corner_min_idx, num_readings - 1))
        corner_max_idx = max(0, min(corner_max_idx, num_readings - 1))
        
        # Get minimum distances in each section
        side_ranges = [r if not math.isinf(r) and not math.isnan(r) else scan.range_max 
                    for r in scan.ranges[side_min_idx:side_max_idx+1]]
        front_ranges = [r if not math.isinf(r) and not math.isnan(r) else scan.range_max 
                      for r in scan.ranges[front_min_idx:front_max_idx+1]]
        corner_ranges = [r if not math.isinf(r) and not math.isnan(r) else scan.range_max 
                       for r in scan.ranges[corner_min_idx:corner_max_idx+1]]
        
        # Calculate minimum distances
        if side_ranges:
            side_dist = min(side_ranges)
        else:
            side_dist = scan.range_max
            
        if front_ranges:
            front_dist = min(front_ranges)
        else:
            front_dist = scan.range_max
            
        if corner_ranges:
            corner_dist = min(corner_ranges)
        else:
            corner_dist = scan.range_max
        
        return side_dist, front_dist, corner_dist
    
    def wall_following_control(self, side_dist, front_dist, corner_dist):
        """Implement wall following control using PID"""
        # Calculate the error (difference from desired distance)
        self.error = self.wall_distance - side_dist
        
        # Calculate PID terms
        proportional = self.kp * self.error
        self.integral += self.ki * self.error * 0.1  # dt = 0.1 seconds
        derivative = self.kd * (self.error - self.prev_error) / 0.1
        
        # Store current error for next iteration
        self.prev_error = self.error
        
        # Calculate angular velocity using PID
        angular_z = proportional + self.integral + derivative
        
        # Limit angular velocity
        angular_z = max(-self.max_rotation, min(self.max_rotation, angular_z))
        
        # Adjust sign based on wall following direction
        if not self.left_wall_following:
            angular_z = -angular_z
        
        # Calculate linear velocity
        # Slow down when approaching obstacles or corners
        linear_x = self.max_speed
        
        # Slow down if too close to front obstacle
        if front_dist < self.min_front_dist:
            linear_x = 0.0
            
            # Turn away from obstacle
            if self.left_wall_following:
                angular_z = -self.max_rotation
            else:
                angular_z = self.max_rotation
        elif front_dist < self.min_front_dist * 2.0:
            # Gradually slow down as we get closer to obstacles
            deceleration_factor = (front_dist - self.min_front_dist) / self.min_front_dist
            linear_x = self.min_speed + (self.max_speed - self.min_speed) * deceleration_factor
            
        # Detect corners or openings and adjust
        corner_threshold = self.wall_distance * 1.5
        if corner_dist > corner_threshold:
            # Detected a corner, opening, or doorway
            self.get_logger().info(f'Detected corner or opening: {corner_dist:.2f}m')
            
            # Turn toward the opening
            if self.left_wall_following:
                angular_z = self.max_rotation * 0.5
            else:
                angular_z = -self.max_rotation * 0.5
                
            # Slow down for the turn
            linear_x = self.min_speed
        
        # Create and return Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        
        return cmd_vel
    
    def check_for_hazard_scan(self):
        """Check if it's time to do a hazard scan"""
        if self.last_hazard_scan_time is None:
            return False
            
        current_time = self.get_clock().now()
        time_since_last_scan = (current_time - self.last_hazard_scan_time).nanoseconds / 1e9
        
        return time_since_last_scan > self.hazard_scan_interval
    
    def check_exploration_timeout(self):
        """Check if exploration time has expired"""
        if self.start_time is None:
            return False
            
        current_time = self.get_clock().now()
        exploration_time = (current_time - self.start_time).nanoseconds / 1e9
        
        return exploration_time > self.explore_timeout
    
    def stop_robot(self):
        """Stop the robot's movement"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def detect_hazard_markers(self):
        """Detect hazard markers using the camera (placeholder)"""
        # In a real implementation, this would process camera data
        # and use computer vision to detect hazard markers
        
        # For demonstration purposes, we'll simulate finding a marker
        if len(self.hazard_markers) < 5:  # If we haven't found all 5 markers yet
            current_position = self.get_current_position()
            
            if current_position is not None:
                # Simulate finding a new marker
                marker_id = len(self.hazard_markers) + 1  # IDs 1-5
                
                # Add small offset so markers aren't exactly at robot position
                position = current_position.position
                position.x += np.random.uniform(-0.3, 0.3)
                position.y += np.random.uniform(-0.3, 0.3)
                
                # Store and publish the marker
                self.hazard_markers[marker_id] = position
                self.publish_hazard_marker(marker_id, position)
                
                self.get_logger().info(f'Detected hazard marker {marker_id}')
                self.publish_status(f"Found hazard marker {marker_id}")
    
    def navigate_return_path(self):
        """Navigate back to home using the recorded path"""
        # For a simplified implementation, we'll just navigate to the home position
        
        if self.home_position is None:
            self.get_logger().error('Home position not set, cannot return')
            return
        
        # Create goal pose
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.pose = self.home_position
        
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
        
        # Send goal
        self._nav_send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_pose,
            feedback_callback=self.nav_feedback_callback
        )
        
        self._nav_send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle the response from the navigation action server"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        self._nav_get_result_future = goal_handle.get_result_async()
        self._nav_get_result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Process feedback from the navigation action"""
        pass  # We don't need to do anything with the feedback
    
    def nav_result_callback(self, future):
        """Handle the result of the navigation action"""
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation completed successfully')
            self.state = RobotState.STOPPED
            self.publish_status("Returned to home position, mission complete")
            
            # Stop the robot
            self.stop_robot()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            # Could implement fallback navigation here
    
    def control_loop(self):
        """Main control loop for the wall-following behavior"""
        
        # Update and publish robot's current position in the path
        current_position = self.get_current_position()
        if current_position is not None and self.state == RobotState.EXPLORING:
            self.add_to_explore_path(current_position)
        
        # State machine
        if self.state == RobotState.WAITING_FOR_START:
            # Waiting for start signal, do nothing
            pass
            
        elif self.state == RobotState.EXPLORING:
            # Check if it's time to do a hazard scan
            if self.check_for_hazard_scan():
                self.state = RobotState.SCANNING_FOR_HAZARDS
                self.last_hazard_scan_time = self.get_clock().now()
                self.publish_status("Scanning for hazards")
                
                # Stop the robot before spinning
                self.stop_robot()
                
                # Start a 360-degree spin
                if not self.start_spin_action():
                    # If spin action failed, resume exploration
                    self.state = RobotState.EXPLORING
            
            # Check if exploration time has expired
            elif self.check_exploration_timeout():
                self.get_logger().info('Exploration timeout reached, returning home')
                self.state = RobotState.RETURNING_HOME
                self.publish_status("Exploration timeout, returning home")
                
                # Prepare for return journey
                self.publish_return_path()
            
            else:
                # Normal wall following behavior
                side_dist, front_dist, corner_dist = self.process_laser_scan()
                
                if side_dist is not None:
                    # Calculate control commands
                    cmd_vel = self.wall_following_control(side_dist, front_dist, corner_dist)
                    
                    # Publish velocity commands
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.publish_status("Exploring: wall following")
                
        elif self.state == RobotState.SCANNING_FOR_HAZARDS:
            # The spin action is handled by callbacks, so we don't need to do anything here
            # Attempt to detect hazard markers during scanning
            self.detect_hazard_markers()
            
        elif self.state == RobotState.RETURNING_HOME:
            # Check if we've already started navigation
            if not hasattr(self, '_nav_send_goal_future') or self._nav_send_goal_future is None:
                # Navigate back to home
                self.navigate_return_path()
            
        elif self.state == RobotState.STOPPED:
            # Robot is stopped at home position, mission complete
            self.stop_robot()
            

def main(args=None):
    rclpy.init(args=args)
    
    wall_follow_node = WallFollowNode()
    
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()