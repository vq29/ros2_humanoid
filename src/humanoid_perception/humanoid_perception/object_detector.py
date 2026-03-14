#!/usr/bin/env python3
"""
Object Detection Node
Detects colored objects (red cube) using OpenCV from the simulated RGB-D camera.
Publishes the 3D pose of the detected object.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import numpy as np

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class ObjectDetector(Node):
    """Detects a red cube from RGB-D camera and publishes its 3D pose."""

    def __init__(self):
        super().__init__('object_detector')

        if not CV_AVAILABLE:
            self.get_logger().error('OpenCV or cv_bridge not available!')
            return

        # Parameters
        self.declare_parameter('target_color_lower', [0, 120, 70])    # HSV lower bound (red)
        self.declare_parameter('target_color_upper', [10, 255, 255])  # HSV upper bound (red)
        self.declare_parameter('min_contour_area', 200)
        self.declare_parameter('camera_frame', 'camera_optical_frame')

        self.color_lower = np.array(
            self.get_parameter('target_color_lower').value, dtype=np.uint8
        )
        self.color_upper = np.array(
            self.get_parameter('target_color_upper').value, dtype=np.uint8
        )
        self.min_area = self.get_parameter('min_contour_area').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # CV Bridge
        self.bridge = CvBridge()

        # Camera intrinsics (populated from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Latest images
        self.color_image = None
        self.depth_image = None

        # Subscribers
        self.color_sub = self.create_subscription(
            Image, '/camera/image', self.color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, '/detected_object_pose', 10
        )

        # Detection timer (10 Hz)
        self.timer = self.create_timer(0.1, self.detect_callback)

        self.get_logger().info('Object detector node initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics from CameraInfo message."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def color_callback(self, msg: Image):
        """Store latest color image."""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert color image: {e}')

    def depth_callback(self, msg: Image):
        """Store latest depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth image: {e}')

    def detect_callback(self):
        """Run object detection and publish pose if object found."""
        if self.color_image is None or self.depth_image is None:
            return
        if self.fx is None:
            return

        # Convert to HSV for color detection
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # Create mask for red color (red wraps around in HSV)
        mask1 = cv2.inRange(hsv, self.color_lower, self.color_upper)
        # Also check the upper red range (170-180)
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_area:
            return

        # Get centroid
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        cx_pixel = int(M['m10'] / M['m00'])
        cy_pixel = int(M['m01'] / M['m00'])

        # Get depth at centroid
        if (cy_pixel >= self.depth_image.shape[0] or
                cx_pixel >= self.depth_image.shape[1]):
            return

        depth = self.depth_image[cy_pixel, cx_pixel]

        # Validate depth
        if np.isnan(depth) or np.isinf(depth) or depth <= 0 or depth > 5.0:
            return

        # Project to 3D using pinhole camera model
        x_3d = (cx_pixel - self.cx) * depth / self.fx
        y_3d = (cy_pixel - self.cy) * depth / self.fy
        z_3d = float(depth)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = x_3d
        pose_msg.pose.position.y = y_3d
        pose_msg.pose.position.z = z_3d
        # Default orientation (no rotation estimation for a cube)
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'Detected object at ({x_3d:.3f}, {y_3d:.3f}, {z_3d:.3f}) '
            f'pixel=({cx_pixel}, {cy_pixel}) depth={depth:.3f}m',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
