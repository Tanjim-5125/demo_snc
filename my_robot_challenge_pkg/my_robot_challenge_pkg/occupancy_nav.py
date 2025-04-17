import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class OccupancyNav(Node):
    def __init__(self):
        super().__init__('occupancy_nav')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())

        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.explore_path_pub = self.create_publisher(Path, '/path_explore', 10)
        self.return_path_pub = self.create_publisher(Path, '/path_return', 10)
        self.hazard_marker_pub = self.create_publisher(Marker, '/hazards', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.exploration_started = True
        self.initial_pose = None
        self.current_pose = None
        self.map_data = None
        self.sent_goals = []
        self.failed_goals = []
        self.goal_in_progress = False
        self.retry_count = 0
        self.max_retries = 3
        self.current_path = Path()

        self.declare_parameter('robot_frame', 'base_link')
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value

        self.create_timer(2.0, self.main_loop)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_hazard_marker(self, x, y, id=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hazards"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.hazard_marker_pub.publish(marker)

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                self.robot_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            return trans.transform.translation
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def map_callback(self, msg):
        self.map_data = msg

    def get_unexplored_regions(self):
        unexplored_points = []
        if self.map_data is None:
            return unexplored_points

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        data = np.array(self.map_data.data).reshape((height, width))

        for y in range(1, height - 1, 5):
            for x in range(1, width - 1, 5):
                if data[y][x] == -1:
                    neighbors = [
                        data[y+dy][x+dx]
                        for dx in [-1, 0, 1]
                        for dy in [-1, 0, 1]
                        if 0 <= x+dx < width and 0 <= y+dy < height and not (dx == 0 and dy == 0)
                    ]
                    if 0 in neighbors:
                        wx = origin.position.x + x * resolution
                        wy = origin.position.y + y * resolution
                        unexplored_points.append((wx, wy))
        return unexplored_points

    def euclidean_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def send_goal(self, x, y, theta=0.0, is_returning=False):
        if self.goal_in_progress:
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        goal_msg.pose = pose

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return

        self.publish_status(f"Sending goal to ({x:.2f}, {y:.2f})")
        self.get_logger().info(f"Sending goal to ({x:.2f}, {y:.2f})")

        if is_returning:
            self.return_path_pub.publish(Path(header=pose.header, poses=[pose]))
        else:
            self.current_path.poses.append(pose)
            self.explore_path_pub.publish(self.current_path)

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self.goal_callback(f, (x, y)))
        self.goal_in_progress = True

    def goal_callback(self, future, sent_goal):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal to {sent_goal} was rejected.")
            self.publish_status(f"Goal to {sent_goal} was rejected.")
            self.failed_goals.append(sent_goal)
            self.goal_in_progress = False
            return

        self.get_logger().info(f"Goal to {sent_goal} accepted.")
        self.publish_status(f"Goal to {sent_goal} accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r: self.result_callback(r, sent_goal, goal_handle))

    def result_callback(self, future, sent_goal, goal_handle):
        status = goal_handle.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal {sent_goal} succeeded.")
            self.publish_status(f"Goal {sent_goal} succeeded.")
            self.sent_goals.append(sent_goal)
            self.retry_count = 0
            self.publish_hazard_marker(sent_goal[0], sent_goal[1], id=len(self.sent_goals))
        else:
            self.get_logger().warn(f"Goal {sent_goal} failed with status: {status}")
            self.publish_status(f"Goal {sent_goal} failed with status: {status}")
            if self.retry_count < self.max_retries:
                self.get_logger().info(f"Retrying goal {sent_goal} ({self.retry_count+1}/{self.max_retries})")
                self.retry_count += 1
                self.send_goal(*sent_goal)
                return
            self.failed_goals.append(sent_goal)
            self.retry_count = 0
        self.goal_in_progress = False

    def spin_in_place(self, duration_sec=2.0):
        msg = Twist()
        msg.angular.z = 0.5
        end_time = self.get_clock().now() + Duration(seconds=duration_sec)
        self.get_logger().info('Spinning in place to discover new areas...')
        self.publish_status('Spinning in place to discover new areas...')
        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_vel_pub.publish(Twist())

    def main_loop(self):

        if not self.exploration_started:
            return
        trans = self.get_robot_pose()
        if trans is None:
            self.get_logger().info('Waiting for current pose...')
            self.publish_status('Waiting for current pose...')
            return

        self.current_pose = trans
        if self.initial_pose is None:
            self.initial_pose = trans

        if self.goal_in_progress:
            return

        unexplored = self.get_unexplored_regions()
        if not unexplored:
            self.get_logger().warn('No unexplored regions found. Trying to spin to discover more...')
            self.publish_status('No unexplored regions found. Spinning...')
            self.spin_in_place(duration_sec=2.0)
            unexplored = self.get_unexplored_regions()
            if not unexplored:
                self.get_logger().warn('Still no regions found. Returning to initial pose.')
                self.publish_status('Returning to initial pose.')
                if self.initial_pose:
                    self.send_goal(self.initial_pose.x, self.initial_pose.y, is_returning=True)
                return

        current_pos = (self.current_pose.x, self.current_pose.y)
        unexplored.sort(key=lambda p: self.euclidean_distance(current_pos, p))
        for point in unexplored:
            if not any(self.euclidean_distance(point, g) < 0.5 for g in self.sent_goals + self.failed_goals):
                self.send_goal(point[0], point[1])
                break

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
