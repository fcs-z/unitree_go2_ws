/*
需求：以动态传参的方式控制go2运动

*/

#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>
#include <sport_model.hpp>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;


class Go2Ctrl: public rclcpp::Node{
public:
    Go2Ctrl(const std::string &node_name):Node(node_name){
      RCLCPP_INFO(this->get_logger(),"go2_ctrl 创建");

      // 设置参数
      this->declare_parameter("sport_api_id", ROBOT_SPORT_API_ID_BALANCESTAND);
      this->declare_parameter("x", 0.0);
      this->declare_parameter("y", 0.0);
      this->declare_parameter("z", 0.0);

      req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
      timer_ = this->create_wall_timer(100ms, std::bind(&Go2Ctrl::on_timer, this));

    }
private:
    void on_timer(){
      // 创建速度指令并发布
      unitree_api::msg::Request request;

      // 设置参数
      auto id = this->get_parameter("sport_api_id").as_int();
      request.header.identity.api_id = id; // 运动模式
      if(id == ROBOT_SPORT_API_ID_MOVE){
        nlohmann::json js;
        js["x"] = this->get_parameter("x").as_double();
        js["y"] = this->get_parameter("y").as_double();
        js["z"] = this->get_parameter("z").as_double();
        request.parameter = js.dump();
      }

      req_pub_->publish(request);
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2Ctrl>("go2_ctrl");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}