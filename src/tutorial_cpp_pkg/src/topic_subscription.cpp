#include "rclcpp/rclcpp.hpp"
#include "tutorial_cpp_pkg/msg/test_topic.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode() : Node("CPP_Topic_Subscription") {
        RCLCPP_INFO(this->get_logger(), "Cpp Topic Subscription");
        sub_ = this->create_subscription<tutorial_cpp_pkg::msg::TestTopic>("cpp_tutorial_topic", 10,
            std::bind(&MyNode::receiveTopic, this, std::placeholders::_1)
            );
        RCLCPP_INFO(this->get_logger(), "Start publish topic...");
    }

private:
    void receiveTopic(const tutorial_cpp_pkg::msg::TestTopic::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Topic received: %ld, %s, %s", msg->msg_int64, msg->msg_bool ? "TRUE" : "FALSE", msg->msg_string.c_str());
    }

private:
    rclcpp::Subscription<tutorial_cpp_pkg::msg::TestTopic>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}