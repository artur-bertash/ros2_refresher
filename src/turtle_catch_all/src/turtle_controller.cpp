#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_turtle_interfaces/msg/list_turtles.hpp"
#include "my_turtle_interfaces/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include <cmath>
#include <string>

class TurtleControllerNode : public rclcpp::Node
{
    public:
        TurtleControllerNode() : Node("turtle_controller") {
            publisher_pose_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            subscriber_turtles_ = this->create_subscription<my_turtle_interfaces::msg::ListTurtles>("/turtle_list", 10, std::bind(&TurtleControllerNode::turtle_list_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TurtleControllerNode::control_loop, this));
            subscriber_pose_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleControllerNode::update_pose_callback, this, std::placeholders::_1));
            client_kill_ = this->create_client<turtlesim::srv::Kill>("/my_kill_turtle");
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_pose_;
        rclcpp::Subscription<my_turtle_interfaces::msg::ListTurtles>::SharedPtr subscriber_turtles_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_pose_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_kill_;

        my_turtle_interfaces::msg::Turtle target_turtle_;
        bool has_target_ = false;

        turtlesim::msg::Pose current_pose_;
        bool pose_received_ = false;

        void turtle_list_callback(const my_turtle_interfaces::msg::ListTurtles::SharedPtr list_turtles) {
            if (list_turtles->turtles.empty()) {
                RCLCPP_WARN(this->get_logger(), "No turtles spawned yet");
            }

            if (!has_target_) {
                target_turtle_ = list_turtles->turtles[0];
                has_target_ = true;
            }
        }

        void control_loop() {
            if (!has_target_) {
                return;
            }

            double dx = target_turtle_.x - current_pose_.x;
            double dy = target_turtle_.y - current_pose_.y;

            double distance = std::sqrt(dx * dx + dy * dy);
            
            double desired_angle = std::atan2(dy, dx);
            double error_angle = desired_angle - current_pose_.theta;

            while (error_angle > M_PI) {
                error_angle -= 2.0 * M_PI;
            }
            while (error_angle < -M_PI)
                error_angle+= 2.0 * M_PI;

            geometry_msgs::msg::Twist cmd;
            double k_gains_linear = 2.0;
            double k_gain_angular = 4.0;

            if (distance < 0.1) {
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                has_target_ = false;  

                auto kill_name_turtle_msg = std::make_shared<turtlesim::srv::Kill::Request>(); 
                kill_name_turtle_msg->name = target_turtle_.name;
                client_kill_->async_send_request(kill_name_turtle_msg);

                RCLCPP_INFO(this->get_logger(), "Target reached!");
                


            } else {
                cmd.linear.x = distance * k_gains_linear;
                cmd.angular.z = error_angle * k_gain_angular;
            }

            publisher_pose_->publish(cmd);

            
        }   

        void update_pose_callback(turtlesim::msg::Pose::SharedPtr new_pose) {
            current_pose_ = *new_pose;
            pose_received_ = true;
        }

        

};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}