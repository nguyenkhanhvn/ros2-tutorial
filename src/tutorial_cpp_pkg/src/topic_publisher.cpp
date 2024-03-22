#include "rclcpp/rclcpp.hpp"
#include "tutorial_cpp_pkg/msg/test_topic.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode() : Node("CPP_Topic_Publisher") {
        RCLCPP_INFO(this->get_logger(), "Cpp Topic Publisher");
        pub_ = this->create_publisher<tutorial_cpp_pkg::msg::TestTopic>("cpp_tutorial_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::publishTopic, this)
            );
        RCLCPP_INFO(this->get_logger(), "Start publish topic...");
    }

private:
    void publishTopic() {
        auto msg = tutorial_cpp_pkg::msg::TestTopic();
        msg.msg_int64 = 111;
        msg.msg_bool = true;
        msg.msg_string = "CPP";
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Topic published");
    }

private:
    rclcpp::Publisher<tutorial_cpp_pkg::msg::TestTopic>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}