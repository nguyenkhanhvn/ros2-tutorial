import rclpy
from rclpy.node import Node
from tutorial_cpp_pkg.msg import TestTopic

class MyNode(Node):
    def __init__(self):
        super().__init__("Python_Topic_Publisher")
        self.declare_parameter(name="param_int64", value=0)
        self.declare_parameter(name="param_bool", value=True)
        self.declare_parameter(name="param_string", value="PYTHON")
        self.publisher_ = self.create_publisher(TestTopic, "py_tutorial_topic", 10)
        self.timer_ = self.create_timer(1.0, self.publish_topic)
        self.get_logger().info("Start publish topic...")
        
    def publish_topic(self):
        msg = TestTopic()
        msg.msg_int64 = self.get_parameter("param_int64").value
        msg.msg_bool = self.get_parameter("param_bool").value
        msg.msg_string = self.get_parameter("param_string").value
        self.publisher_.publish(msg)
        self.get_logger().info("Topic published!")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()