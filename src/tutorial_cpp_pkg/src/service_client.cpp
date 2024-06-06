#include "rclcpp/rclcpp.hpp"
#include "tutorial_cpp_pkg/srv/test_service.hpp"

class MyNode: public rclcpp::Node
{
public:
    MyNode() : Node("CPP_Service_Client") {
        RCLCPP_INFO(this->get_logger(), "Cpp Service Client");
        this->declare_parameter("param1", 1.1);
        this->declare_parameter("param2", 2.2);
        thread_ = std::thread(std::bind(&MyNode::callService, this, std::vector<double>{this->get_parameter("param1").as_double(), this->get_parameter("param2").as_double()}));
        
        RCLCPP_INFO(this->get_logger(), "Cpp Service Client inited");
    }

private:
    void callService(std::vector<double> datas) {
        RCLCPP_INFO(this->get_logger(), "callService");
        auto client = this->create_client<tutorial_cpp_pkg::srv::TestService>("cpp_tutorial_service");
        while(!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for Server...");
        }

        auto request = std::make_shared<tutorial_cpp_pkg::srv::TestService::Request>();
        request->datas = datas;

        auto future = client->async_send_request(request);

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %lf", response->sum);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    std::thread thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}