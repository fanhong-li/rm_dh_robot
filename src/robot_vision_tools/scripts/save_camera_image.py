#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.image_saved = False
        self.get_logger().info('等待相机图像...')

    def image_callback(self, msg):
        if not self.image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite('test_image.png', cv_image)
                self.get_logger().info('图片已保存为 test_image.png')
                self.image_saved = True
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f'保存图片失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()

if __name__ == '__main__':
    main() 