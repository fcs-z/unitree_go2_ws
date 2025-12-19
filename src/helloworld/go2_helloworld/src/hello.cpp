#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>

using namespace std::chrono_literals;
 
class HelloWorld: public rclcpp::Node{
public:
    HelloWorld(const std::string &node_name):Node(node_name){
       pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
       timer_ = this->create_wall_timer(
           1s,
           std::bind(&HelloWorld::timer_callback, this)
       );
      RCLCPP_INFO(this->get_logger(), "Hello World node has been started.");
    }
private:
    void timer_callback(){
        unitree_api::msg::Request request;
        request.header.identity.api_id = 1016;  // 打招呼的协议码
        pub_->publish(request);
    }
    
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorld>("hello_world");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}