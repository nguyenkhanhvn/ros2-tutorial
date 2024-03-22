import rclpy
from rclpy.node import Node
from functools import partial
from tutorial_cpp_pkg.srv import TestService

class MyNode(Node):
    def __init__(self):
        super().__init__("Python_Service_Client")
        self.get_logger().info("Start client...")
        # self.callService([2.3, 4.5])
        
    def callService(self, datas):
        client = self.create_client(TestService, "py_tutorial_service")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")
            
        request = TestService.Request()
        request.datas = datas
        
        self.get_logger().info("send request")
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_callService))
        
    def callback_callService(self, future):
        try:
            response = future.result()
            self.get_logger().info("result: " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call Failed %r" % (e,))
            

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()