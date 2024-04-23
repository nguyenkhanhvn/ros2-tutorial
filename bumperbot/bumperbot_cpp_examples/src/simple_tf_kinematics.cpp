#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <my_robot_interfaces/srv/get_transform.hpp>

using namespace std::chrono_literals;

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics() : Node("simple_tf_kinematics")
    {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        static_transform_stamped_.header.stamp = get_clock()->now();
        static_transform_stamped_.header.frame_id = "bumperbot_base";
        static_transform_stamped_.child_frame_id = "bumperbot_top";
        static_transform_stamped_.transform.translation.x = 0.0;
        static_transform_stamped_.transform.translation.y = 0.0;
        static_transform_stamped_.transform.translation.z = 0.3;
        static_transform_stamped_.transform.rotation.x = 0.0;
        static_transform_stamped_.transform.rotation.y = 0.0;
        static_transform_stamped_.transform.rotation.z = 0.0;
        static_transform_stamped_.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(static_transform_stamped_);
        
        RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between " <<
                               static_transform_stamped_.header.frame_id << " and " << static_transform_stamped_.child_frame_id);

        timer_ = create_wall_timer(0.1s, [&]() {
            dynamic_transform_stamped_.header.stamp = get_clock()->now();
            dynamic_transform_stamped_.header.frame_id = "odom";
            dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
            dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
            dynamic_transform_stamped_.transform.translation.y = 0.0;
            dynamic_transform_stamped_.transform.translation.z = 0.0;

            tf2::Quaternion q = last_orientation_ * orientation_increment_;
            q.normalize();
            dynamic_transform_stamped_.transform.rotation.x = q.x();
            dynamic_transform_stamped_.transform.rotation.y = q.y();
            dynamic_transform_stamped_.transform.rotation.z = q.z();
            dynamic_transform_stamped_.transform.rotation.w = q.w();

            dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
        

            last_x_ = dynamic_transform_stamped_.transform.translation.x;
            last_orientation_ = q;
        });

        get_transform_srv_ = create_service<my_robot_interfaces::srv::GetTransform>("get_transform", std::bind(&SimpleTfKinematics::getTransformCallback, this, std::placeholders::_1, std::placeholders::_2));

        last_orientation_.setRPY(0, 0, 0);
        orientation_increment_.setRPY(0, 0, 0.05);
    }

    bool getTransformCallback(const std::shared_ptr<my_robot_interfaces::srv::GetTransform::Request> req, const std::shared_ptr<my_robot_interfaces::srv::GetTransform::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Request transform between " << req->frame_id << " and " << req->child_frame_id);
        geometry_msgs::msg::TransformStamped requested_transform;
        try {
            requested_transform = tf_buffer_->lookupTransform(req->frame_id, req->child_frame_id, tf2::TimePointZero);
        }
        catch(tf2::TransformException &ex)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "An error occurred while transforming from " << req->frame_id << " and " << req->child_frame_id << ": " << ex.what());
            res->success = false;
            return true;
        }
        
        res->transform = requested_transform;
        res->success = true;
        return true;
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<my_robot_interfaces::srv::GetTransform>::SharedPtr get_transform_srv_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    double x_increment_ = 0.05;
    double last_x_ = 0.0;

    tf2::Quaternion last_orientation_;
    tf2::Quaternion orientation_increment_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}