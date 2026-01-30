from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class MocapSubscriber(Node):
    def __init__(self, node: Node,topic: str = "/mocap/rigidbody1/marker0", qos: int = 10):
        self.node = node
        self.received = False
        self.latest = None
        self.history = []
        self.subscriber = node.create_subscription(PointStamped, topic, self.callback, qos)
        
    def callback(self, msg:PointStamped):
        self.latest = msg
        self.history.append(msg)
        self.received = True
    
    def has_received(self) -> bool:
        return self.received
    
    def consume_latest(self) -> PointStamped | None:
        # 最新データを取得し，受信フラグをリセット
        if not self.received:
            return None
        self.received = False
        return self.latest
    
    
    