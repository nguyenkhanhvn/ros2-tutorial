#include "rclcpp/rclcpp.hpp"
#include "tutorial_cpp_pkg/srv/test_service.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode() : Node("CPP_Service_Server") {
        RCLCPP_INFO(this->get_logger(), "Cpp Service Server");
        server_ = this->create_service<tutorial_cpp_pkg::srv::TestService>("cpp_tutorial_service",
            std::bind(&MyNode::callbackService, this, std::placeholders::_1, std::placeholders::_2)
            );
        RCLCPP_INFO(this->get_logger(), "Start service...");
    }

private:
    void callbackService(const tutorial_cpp_pkg::srv::TestService::Request::SharedPtr request,
                         const tutorial_cpp_pkg::srv::TestService::Response::SharedPtr response) {
        response->sum = 0.0;
        for(double data: request->datas) {
            response->sum += data;
        }
        RCLCPP_INFO(this->get_logger(), "Result: %lf", response->sum);
    }

private:
    rclcpp::Service<tutorial_cpp_pkg::srv::TestService>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}