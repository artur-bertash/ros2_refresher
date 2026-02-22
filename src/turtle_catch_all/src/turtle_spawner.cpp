#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_turtle_interfaces/msg/list_turtles.hpp"
#include "my_turtle_interfaces/msg/turtle.hpp"

#include <random>


class TurtleSpawnerNode : public rclcpp::Node
{
    public:
        TurtleSpawnerNode() : Node("turtle_spawner") {
            client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            counter_ = 2;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&TurtleSpawnerNode::timer_callback, this));
            publisher_ = this->create_publisher<my_turtle_interfaces::msg::ListTurtles>("turtle_list", 10);
        }
        
        


    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<my_turtle_interfaces::msg::ListTurtles>::SharedPtr publisher_;
        std::vector<my_turtle_interfaces::msg::Turtle> vector_turtles;

    

        int counter_;
        void client_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future, double x, double y, std::string name) {
            auto response = future.get();
            auto turtle = my_turtle_interfaces::msg::Turtle();
            turtle.name = name;
            turtle.x = x;
            turtle.y = y;
            vector_turtles.push_back(turtle);

            auto list_turtles = my_turtle_interfaces::msg::ListTurtles();
            list_turtles.turtles = vector_turtles;
            publisher_->publish(list_turtles);
            
            RCLCPP_INFO(this->get_logger(), "Turtle %s spawned at (%.2f, %.2f)", name.c_str(), x, y);
        }

        void spawn_turtle() {
            while (!client_->wait_for_service(std::chrono::milliseconds(2000))) {
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

            auto future = client_->async_send_request(random_turtle, 
                [this, x = random_turtle->x, y = random_turtle->y, name = random_turtle->name](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                    client_callback(future, x, y, name);
            });

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