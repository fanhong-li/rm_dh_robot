#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_custom')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('camera_frame', 'camera_color_frame')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.get_logger().info(f'寻找marker ID: {self.marker_id}, 尺寸: {self.marker_size}m')
        self.get_logger().info(f'OpenCV版本: {cv2.__version__}')
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            self.use_new_api = True
            self.get_logger().info('使用新版OpenCV API')
        except AttributeError:
            try:
                self.parameters = cv2.aruco.DetectorParameters_create()
                self.use_new_api = False
                self.get_logger().info('使用旧版OpenCV API (DetectorParameters_create)')
            except AttributeError:
                try:
                    self.parameters = cv2.aruco.DetectorParameters()
                    self.use_new_api = False
                    self.get_logger().info('使用旧版OpenCV API (DetectorParameters)')
                except AttributeError:
                    self.parameters = None
                    self.use_new_api = False
                    self.get_logger().info('使用最老版本OpenCV API')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_single/pose', 10)
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info('收到相机内参，开始处理图像')
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('等待相机内参...', throttle_duration_sec=2.0)
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f'处理图像，尺寸: {cv_image.shape}', throttle_duration_sec=2.0)
            if self.use_new_api:
                corners, ids, rejected = self.detector.detectMarkers(cv_image)
            else:
                if self.parameters is not None:
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        cv_image, self.aruco_dict, parameters=self.parameters)
                else:
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        cv_image, self.aruco_dict)
            if ids is not None:
                self.get_logger().info(f'检测到markers: {ids.flatten()}', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('未检测到任何marker', throttle_duration_sec=2.0)
            if ids is not None and self.marker_id in ids.flatten():
                idx = np.where(ids.flatten() == self.marker_id)[0][0]
                marker_corners = corners[idx]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [marker_corners], self.marker_size, self.camera_matrix, self.dist_coeffs)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                self.publish_tf(rvec, tvec, msg.header.stamp)
                self.publish_pose(rvec, tvec, msg.header.stamp)
                self.get_logger().info(f'检测到marker {self.marker_id} at位置: {tvec}', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')

    def publish_tf(self, rvec, tvec, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.marker_frame
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self, rvec, tvec, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 