#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



class TurtleControllerNode : public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller") {
             publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}