import rclpy
from rclpy.node import Node
from tutorial_cpp_pkg.srv import TestService

class MyNode(Node):
    def __init__(self):
        super().__init__("Python_Service_Server")
        self.server_ = self.create_service(TestService, "py_tutorial_service", self.callback_service)
        self.get_logger().info("Start service...")
        
    def callback_service(self, req, res):
        res.sum = 0.0
        for data in req.datas:
            res.sum += data
        self.get_logger().info("Service result: " + str(res.sum))
        return res

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()