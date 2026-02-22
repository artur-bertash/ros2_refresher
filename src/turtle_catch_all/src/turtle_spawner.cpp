#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>


class TurtleSpawnerNode : public rclcpp::Node
{
    public:
        TurtleSpawnerNode() : Node("turtle_spawner") {
            client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            counter_ = 0;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&TurtleSpawnerNode::timer_callback, this));
        }
        
        


    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;


        int counter_;
        void client_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle #%d is spawned", counter_);
        }

        void spawn_turtle() {
            while (!client_->wait_for_service(std::chrono::milliseconds(500))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for /spawn server");
            }
            auto random_turtle = std::make_shared<turtlesim::srv::Spawn::Request>();

            //random numebers
            
            auto gen = std::mt19937(std::random_device{}());
            auto dist = std::uniform_real_distribution<double>(0.0, 11.0);


            random_turtle->x = dist(gen);
            random_turtle->y = dist(gen); 
            random_turtle->name = "turtle" + std::to_string(counter_);
            
            counter_++;

            auto future = client_->async_send_request(random_turtle, std::bind(&TurtleSpawnerNode::client_callback, this, std::placeholders::_1));

        }
        void timer_callback() {
            this->spawn_turtle();
        }
    
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;                           
}