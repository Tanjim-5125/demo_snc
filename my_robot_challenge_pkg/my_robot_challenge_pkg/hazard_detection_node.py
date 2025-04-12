import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
import math


class HazardMarkerNode(Node):

    def __init__(self):
        super().__init__('hazard_marker_node')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.object_sub = self.create_subscription(
            String,
            '/detected_object',
            self.object_callback,
            10
        )

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(
            Marker,
            '/hazard_markers',
            10
        )

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store latest scan data
        self.latest_scan = None
        # To track last published object to avoid duplicates
        self.last_published_obj = None

    def scan_callback(self, msg):
        self.latest_scan = msg

    def object_callback(self, msg):
        if self.latest_scan is None:
            return

        detected_obj = msg.data

        # Avoid duplicate processing
        if detected_obj == self.last_published_obj:
            return

        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        # Look for the closest obstacle within 1 meter
        min_range = float('inf')
        min_index = -1

        for i, r in enumerate(ranges):
            if 0.05 < r < 1.0:  # ignore very close and too far
                if r < min_range:
                    min_range = r
                    min_index = i

        if min_index == -1:
            self.get_logger().info('No hazard within 1 meter.')
            return

        angle = angle_min + min_index * angle_increment
        x = min_range * math.cos(angle)
        y = min_range * math.sin(angle)

        # Create PoseStamped in laser frame
        pose = PoseStamped()
        pose.header.frame_id = self.latest_scan.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Transform to map frame
        target_frame = 'map'
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time()
            )
            pose_map = tf2_geometry_msgs.do_transform_pose(pose, transform)
        except Exception as e:
            self.get_logger().error(f'Failed to transform pose: {e}')
            return

        # Generate unique ID from object label (e.g., hash or string ID)
        obj_id = abs(hash(detected_obj)) % 10000

        # Build the Marker message for the hazard marker
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hazards"
        marker.id = obj_id  # Use the detected object id as the marker id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose_map.pose.position.x
        marker.pose.position.y = pose_map.pose.position.y
        marker.pose.position.z = pose_map.pose.position.z
        marker.pose.orientation.w = 1.0

        # Set the marker scale (adjust based on your application)
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Set marker color (red, fully opaque)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Infinite lifetime (lifetime = 0)
        marker.lifetime.sec = 0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published hazard marker for: {detected_obj}")
        self.last_published_obj = detected_obj


def main(args=None):
    rclpy.init(args=args)
    node = HazardMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
