#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_matrix

class CheckerboardDetector(Node):
    def __init__(self):
        super().__init__('checkerboard_detector')
        
        # 声明参数
        self.declare_parameter('board_width', 9)
        self.declare_parameter('board_height', 6)
        self.declare_parameter('square_size', 0.02)  # 2cm
        self.declare_parameter('camera_frame', 'camera_color_frame')
        self.declare_parameter('target_frame', 'calibration_target')
        
        # 获取参数
        self.board_width = self.get_parameter('board_width').value
        self.board_height = self.get_parameter('board_height').value
        self.square_size = self.get_parameter('square_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        
        # 棋盘格参数
        self.board_size = (self.board_width, self.board_height)
        
        # 创建3D点 (棋盘格在其自身坐标系中的坐标)
        self.object_points = np.zeros((self.board_width * self.board_height, 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:self.board_width, 0:self.board_height].T.reshape(-1, 2)
        self.object_points *= self.square_size
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        
        self.get_logger().info(f'Checkerboard detector started. Looking for {self.board_width}x{self.board_height} board with {self.square_size}m squares')

    def camera_info_callback(self, msg):
        """获取相机内参"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        """处理图像，检测棋盘格"""
        if self.camera_matrix is None:
            return
            
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 检测棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            
            if ret:
                # 精确化角点
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # 求解PnP问题，获得旋转和平移向量
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points, corners2, self.camera_matrix, self.dist_coeffs)
                
                if success:
                    # 发布TF变换
                    self.publish_transform(rvec, tvec, msg.header.stamp)
                    
                    # 可选：在图像上绘制检测结果
                    # cv2.drawChessboardCorners(cv_image, self.board_size, corners2, ret)
                    # cv2.imshow('Checkerboard Detection', cv_image)
                    # cv2.waitKey(1)
                    
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def publish_transform(self, rvec, tvec, timestamp):
        """发布TF变换"""
        # 转换旋转向量到旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # 创建4x4变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = tvec.flatten()
        
        # 转换到四元数
        quaternion = quaternion_from_matrix(transform_matrix)
        
        # 创建TF消息
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.target_frame
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    detector = CheckerboardDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 