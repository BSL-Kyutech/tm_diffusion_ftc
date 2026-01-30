# press40_hub(pressure40)を定義
# 内部で40次元の制御入力を40→(10×4)に分割してPressure12をPublish
from numpy.typing import NDArray
import rclpy
import numpy as np
from rclpy.node import Node
from tenpa_msgs.msg import Pressure12

# rclpy.init()
# node = Node('publisher')
# publisher_list = []

# for i in range(4):
#     publisher = node.create_publisher(
#         Pressure12,
#         'tenpa/pressure/desired' + str(i),
#         10
#     )
#     publisher_list.append(publisher)
    
# def press40_hub(input40:list[int]):
#     for i in range(4):
#         input12 = np.zeros(12) #1つの層に12個の入力
#         #input40を10ごとに分割
#         input12[0:10] = input40[0+10*i : 10+10*i]
#         msg = Pressure12()
#         msg.pressure = np.uint16(input12)
#         publisher_list[i].publish(msg)

class PressurePublisher(Node):
    def __init__(self,node_name:str = "pressure_publisher", base_topic:str="/tenpa/pressure/desired",
                 num_layers: int =4, per_layer_channels: int = 10, qos_depth: int = 10,) -> None:
        super().__init__(node_name)
        self.num_layers = num_layers
        self.per_layer_channnels = per_layer_channels
        self.base_topic = base_topic
        
        self.publisher_list = [
            self.create_publisher(Pressure12, f"{base_topic}{i}",qos_depth)
            for i in range(num_layers)
        ]
    
    def publish40(self, input40:list[int]) -> None:
        # input40をpublishする
        expected = self.num_layers * self.per_layer_channnels
        if len(input40) != expected:
            raise ValueError(f"input0 length must be {expected}, got {len(input40)}")
        
        for i in range(self.num_layers):
            start = i * self.per_layer_channnels
            end = start + self.per_layer_channnels
            
            input12 = np.zeros(12, dtype=np.uint16)
            input12[0:self.per_layer_channnels] = np.asarray(input40[start:end], dtype=np.uint16)
            
            msg = Pressure12()
            msg.pressure = np.uint16(input12)
            self.publisher_list[i].publish(msg)