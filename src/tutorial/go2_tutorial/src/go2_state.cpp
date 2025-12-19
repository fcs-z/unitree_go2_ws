/*
需求：订阅里程计消息，当机器人位移超出指定阀值，输出机器人此刻的坐标。
实现流程：
    1.订阅里程计消息；
    2.在订阅方的回调函数中，计算当前机器人位置与上一个记录点的距离，
        如果距离大于阀值就输出坐标，并更新记录点。
    第一个记录点如何生成？
    当第一次订阅到里程计消息时，就为记录点赋值。
*/


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
 
class Go2State: public rclcpp::Node{
public:
    Go2State(const std::string &node_name):Node(node_name){
        RCLCPP_INFO(this->get_logger(), "Go2State 创建");

        is_first = true;
        last_x = last_y = 0.0;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&Go2State::odom_cb, this, std::placeholders::_1)
        );

        this->declare_parameter("distance", 0.5);
    }
private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        // 初始化第一个记录点数据
        // 获取当前坐标
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;

        // 如果是第一次订阅到该消息，那么就赋值
        if(is_first){
            last_x = x;
            last_y = y;
            is_first = false;
            RCLCPP_INFO(this->get_logger(), "起点坐标：(%.2f, %.2f)", last_x, last_y);
            return;
        }

        //计算是否超出阀值
        double distance_x = x - last_x;
        double distance_y = y - last_y;
        // 计算距离
        double distance = sqrt(pow(distance_x,2) + pow(distance_y,2));
        auto dis = this->get_parameter("distance").as_double();
        if(distance > dis){
            RCLCPP_INFO(this->get_logger(), "坐标：(%.2f, %.2f)", x, y);
            // 更新记录点
            last_x = x;
            last_y = y;
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double last_x, last_y;
    bool is_first;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2State>("go2_state");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}