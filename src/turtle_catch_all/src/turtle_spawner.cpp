#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_turtle_interfaces/msg/list_turtles.hpp"
#include "my_turtle_interfaces/msg/turtle.hpp"
#include "turtlesim/srv/kill.hpp"
#include <algorithm>
#include <random>


class TurtleSpawnerNode : public rclcpp::Node
{
    public:
        TurtleSpawnerNode() : Node("turtle_spawner") {
            client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
            counter_ = 2;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&TurtleSpawnerNode::timer_callback, this));
            publisher_ = this->create_publisher<my_turtle_interfaces::msg::ListTurtles>("turtle_list", 10);
            server_ = this->create_service<turtlesim::srv::Kill>("/my_kill_turtle", std::bind(&TurtleSpawnerNode::kill_turtle_server_callback, this, std::placeholders::_1, std::placeholders::_2));
            client_kill_ = this->create_client<turtlesim::srv::Kill>("/kill");
        }
        
        


    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<my_turtle_interfaces::msg::ListTurtles>::SharedPtr publisher_;
        std::vector<my_turtle_interfaces::msg::Turtle> vector_turtles;
        rclcpp::Service<turtlesim::srv::Kill>::SharedPtr server_;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_kill_;
    

        int counter_;
        void client_callback(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future, double x, double y, std::string name) {
            (void)future;
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
            while (!client_->wait_for_service(std::chrono::milliseconds(1000))) {
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


        void kill_turtle_server_callback(const turtlesim::srv::Kill::Request::SharedPtr req, 
                                   turtlesim::srv::Kill::Response::SharedPtr res ) {
            (void)res;
            while (!client_kill_->wait_for_service(std::chrono::milliseconds(2000))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for /Kill server");
            }

            auto kill_name_msg = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_name_msg->name = req->name;

            
            auto future = client_kill_->async_send_request(kill_name_msg, 
                [this, name = req->name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future) {
                    service_kill_callback(future, name);
                });
            
            

        }

        void service_kill_callback(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future, std::string name) {
            (void)future;
            vector_turtles.erase(
                std::remove_if(
                    vector_turtles.begin(),
                    vector_turtles.end(),
                    [&](const my_turtle_interfaces::msg::Turtle & t) {
                        return t.name == name;
                    }),
                vector_turtles.end()
            );

            RCLCPP_INFO(this->get_logger(), "The turtle is killed");

            auto list_msg = my_turtle_interfaces::msg::ListTurtles();
            list_msg.turtles = vector_turtles;
            publisher_->publish(list_msg);
        }
    
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;                           
}