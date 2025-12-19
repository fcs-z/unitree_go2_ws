/*
    需求：订阅twist消息，并将之转换为个go2所需的request消息，以控制机器狗运动
    实现：
      1. 创建一个Request的发布对象
      2. 创建twist订阅对象
      3. 在回调函数中实现消息的转换以及发布。
*/

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

// 自定义节点类
class TwistBridge :public rclcpp::Node{
public:
  TwistBridge():Node("twist_brige")
  {
      RCLCPP_INFO(this->get_logger(),"TwistBridge创建成功");
      // 1. 创建一个Request的发布对象
      request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
      // 2. 创建twist订阅对象
      twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,
        std::bind(&TwistBridge::twist_cb,this,_1));
  }

private:
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist){
    // 3. 在回调函数中实现消息的转换以及发布
    unitree_api::msg::Request request_msg;

    // 转换
    // 获取 twist 消息的线速度和角速度
    double x = twist->linear.x;
    double y = twist->linear.y;
    double z = twist->angular.z;
    // 默认api_id 平衡站立
    auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
    // 运动状态
    if (x != 0 || y != 0 || z != 0){
      api_id = ROBOT_SPORT_API_ID_MOVE;
      // 设置参数 -- 字符串样式的速度指令
      nlohmann::json js;
      js["x"] = x;
      js["y"] = y;
      js["z"] = z;
      request_msg.parameter = js.dump();
    }
    request_msg.header.identity.api_id = api_id;

    // 发布Request消息
    request_pub_->publish(request_msg);
  }

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<TwistBridge>());
  rclcpp::shutdown();
  return 0;
}