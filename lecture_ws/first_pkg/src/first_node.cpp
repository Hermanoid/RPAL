#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_node");

    // Add your code here
    RCLCPP_INFO(node->get_logger(), "Hello, world!");

    // Example code: publishing a message
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [publisher]() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        publisher->publish(message);
    };
    auto timer = node->create_wall_timer(std::chrono::seconds(1), timer_callback);

    rclcpp::spin(node);

    return 0;
}