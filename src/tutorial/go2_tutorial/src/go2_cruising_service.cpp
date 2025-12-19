/*
需求：处理客户端提交的数据（0或非0），
    如果是0，停止巡航，如果非，开始巡航
    不过提交的什么数据都需要响应机器人的位置信息。
流程：
    1.创建服务端；
    2.回调函数处理请求，分情况处理（控制机器人巡航），最后响应结果。   
*/

#include <rclcpp/rclcpp.hpp>
#include <go2_tutorial_inter/srv/cruising.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sport_model.hpp>


using namespace std::chrono_literals;

class Go2CrusingService: public rclcpp::Node{
public:
    Go2CrusingService(const std::string &node_name):Node(node_name){
         RCLCPP_INFO(this->get_logger(),"Go2CruisingService 创建");

        // 创建服务端
        cru_service_ = this->create_service<go2_tutorial_inter::srv::Cruising>("cruising", 
            std::bind(&Go2CrusingService::cru_cb, this, std::placeholders::_1, std::placeholders::_2));

        // 通过订阅里程计获取机器人的位置信息
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&Go2CrusingService::odom_cb, this, std::placeholders::_1));

        // 创建远程参数客户端
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "go2_ctrl");
        // 让客户端连接服务端
        while (!param_client_->wait_for_service(1s)) {
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "参数服务端go2_ctrl连接中...");
        }
        RCLCPP_INFO(this->get_logger(), "成功连接到参数服务端go2_ctrl!");

        // 参数
        this->declare_parameter("x", 0.1);
        this->declare_parameter("y", 0.0);
        this->declare_parameter("z", 0.5);
    }
private:
    // void cru_cb(
    //     const std::shared_ptr<go2_tutorial_inter::srv::Cruising::Request> request,
    //     const std::shared_ptr<go2_tutorial_inter::srv::Cruising::Response> response){    
    // }
    void cru_cb(const go2_tutorial_inter::srv::Cruising::Request::SharedPtr request,
                const go2_tutorial_inter::srv::Cruising::Response::SharedPtr response){
        // 处理请求
        auto flag = request->flag;
        // 向go2_ctrl节点设置参数, 如果是0, api设置为stopmove, 否则设置为sport
        int32_t id;
        if(flag != 0){
            id = ROBOT_SPORT_API_ID_MOVE;
            RCLCPP_INFO(this->get_logger(),"开始巡航...");
            
        }else{
            id = ROBOT_SPORT_API_ID_STOPMOVE;
            RCLCPP_INFO(this->get_logger(),"停止巡航...");
            
        }
        // 向巡航节点设置参数
        param_client_->set_parameters({
            // rclcpp::Parameter("x", 0.1),
            // rclcpp::Parameter("y", 0.0),
            // rclcpp::Parameter("z", 0.5),
            this->get_parameter("x"),   // 本节点的参数
            this->get_parameter("y"),
            this->get_parameter("z"),
            rclcpp::Parameter("api", id) // go2_ctrl节点的sport_api_id参数
        });

        // 生成响应
        response->point = current_point_;
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        // 获取机器人的位置信息
        current_point_ = odom->pose.pose.position;
    }

    rclcpp::Service<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_service_;
    geometry_msgs::msg::Point current_point_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // 参数客户端
     rclcpp::AsyncParametersClient::SharedPtr param_client_;
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CrusingService>("go2_crusing_service");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}