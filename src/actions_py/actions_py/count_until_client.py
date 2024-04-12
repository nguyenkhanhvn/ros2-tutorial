import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client_py")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        self.get_logger().info("Action server has been started")
        
    def send_goal(self, target_number, period):
        # Wait for the server
        self.count_until_client_.wait_for_server(10)
        
        #create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        
        # Send the goal
        self.get_logger().info("Sending goal")
        self.count_until_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)
        
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)
        
    def cancel_goal(self):
        self.get_logger().info("Sending a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()
        
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got Rejected")
            
    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        result = future.result().result
        self.get_logger().info("Result: " + str(result.reached_number))
        
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Feedback: " + str(number))
        
            
            

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(16, 1.0)
    node.send_goal(8, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()