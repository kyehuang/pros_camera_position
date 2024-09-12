import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedImageSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_sub_pub')
        
        # 訂閱壓縮的圖像消息
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10)
        
        # 建立一個新的 Image 發布者來發布解壓後的圖像
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # 初始化 CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received compressed image')
        
        # 解壓縮影像
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 將解壓後的 OpenCV 圖像轉換為 ROS2 的 Image 消息格式
        image_message = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
        
        # 發布未壓縮的影像
        self.publisher_.publish(image_message)
        self.get_logger().info('Published raw image')

def main(args=None):
    rclpy.init(args=args)
    
    # 節點初始化
    node = CompressedImageSubscriberPublisher()
    
    rclpy.spin(node)

    # 清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
