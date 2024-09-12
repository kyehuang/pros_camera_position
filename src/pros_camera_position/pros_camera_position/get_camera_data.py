import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from camera_coordinate import convert_camera_coordinates
import pickle
import numpy as np

class StringSubscriber(Node):
    def __init__(self):
        super().__init__('string_subscriber')
        
        # 訂閱 std_msgs/String 類型的消息
        self.subscription = self.create_subscription(
            String,
            'camera1_data',  # 將 'topic_name' 替換為你要訂閱的話題名稱
            self.listener_callback,
            10)
        self.subscription  # 防止未被使用的警告


        with open("./camera_data/camera_1.pkl", "rb") as infile:
            camera = pickle.load(infile)
            self.camera_matrix = camera["camera_matrix"]            
            self.extrinsic_matrix = camera["extrinsic_matrix"]
            self.rotation_matrix = camera["rotation_matrix"]
        self.camera_transfrom = np.array( [[0, 4, 9]] )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received raw message: "{msg.data}"')
        
        try:
            # 將消息內容轉換為 JSON 格式
            data = json.loads(msg.data)
            
            # 提取 JSON 數據中的字段
            u = data['u']
            v = data['v']
            depth = data['depth']
            object_name = data['objectName']
            
            # 打印解析後的數據
            self.get_logger().info(f'u: {u}, v: {v}, depth: {depth}, objectName: {object_name}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in JSON data: {e}")
        
        self.get_positon(u, v, depth)

    
    def get_positon(self, u, v, depth):        
        convert_camera = convert_camera_coordinates(self.camera_matrix, self.extrinsic_matrix, self.rotation_matrix, self.camera_transfrom)
        scream_points = np.array(
            [[u, v, 1]]
        )        
        result = convert_camera.screen_point_to_world_point(scream_points, depth)
        
        self.get_logger().info(f"result: {result}")

        


def main(args=None):
    rclpy.init(args=args)

    # 初始化節點
    node = StringSubscriber()
    
    # 運行節點
    rclpy.spin(node)

    # 清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
