import rclpy
from rclpy.node import Node
from tutorial_cpp_pkg.msg import TestTopic

class MyNode(Node):
    def __init__(self):
        super().__init__("Python_Topic_Subscription")
        self.publisher_ = self.create_subscription(TestTopic, "py_tutorial_topic", self.receive_topic, 10)
        self.get_logger().info("Start receive topic...")
        
    def receive_topic(self, msg):
        self.get_logger().info("Topic received: " + str(msg.msg_int64) + ", " + str(msg.msg_bool) + ", " + msg.msg_string)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()