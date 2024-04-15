import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("Python_Number_Publisher")
        self.get_logger().info("On constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("On configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number
        )
        self.number_timer_.cancel()
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("On activate")
        self.number_timer_.reset()
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("On deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state)
    
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("On cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("On shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("On error")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        
        return TransitionCallbackReturn.SUCCESS
        
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.get_logger().info("publish_number " + str(self.number_))
        self.number_ +=1
            

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()