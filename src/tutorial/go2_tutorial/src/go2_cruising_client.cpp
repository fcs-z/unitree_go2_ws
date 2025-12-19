/*
需求：客户端向服务端发送整型数据，并接收服务端的响应结果。
流程：
    1.判断提交的数据是否合法；
    2.R0S2初始化；
    3.创建自定义节点类对象；
    4.连接服务端；
    5.连接成功后需要向服务端发送数据；
    6.处理响应结果；
    7.资源释放。
*/

#include <rclcpp/rclcpp.hpp>
#include <go2_tutorial_inter/srv/cruising.hpp>

using namespace std::chrono_literals;
 
class Go2CrusingClient: public rclcpp::Node{
public:
    Go2CrusingClient(const std::string &node_name):Node(node_name){
       cru_client_ = this->create_client<go2_tutorial_inter::srv::Cruising>("cruising");
    }
    
    bool connect_service(){
        while(!cru_client_->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务端连接中...");
        }
        RCLCPP_INFO(this->get_logger(), "成功连接到服务端!");
        return true;
    }

    // 发送请求并返回响应结果
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::FutureAndRequestId send_request(int32_t flag){
        // 创建请求对象
        // go2_tutorial_inter::srv::Cruising::Request::SharedPtr req_;  // 只声明，没有创建对象,错误的
        // req_->flag = flag;
        auto req_ = std::make_shared<go2_tutorial_inter::srv::Cruising::Request>();
        req_->flag = flag;
        return cru_client_->async_send_request(req_);
    }
private:
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_client_;
};
 
int main(int argc, char* argv[]){
    if(argc != 2){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "参数不正确！需要一个整数参数，0表示停止巡航，非0表示开始巡航");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CrusingClient>("go2_crusing_client");
    
    // 连接服务端
    auto flag = node->connect_service();
    if(!flag){
        return 1;
    }

    // 处理响应结果
    auto future = node->send_request(atoi(argv[1]));
    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(node->get_logger(), "响应成功 !");
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "机器人坐标: x=%.2f, y=%.2f, z=%.2f", response->point.x, response->point.y, response->point.z); 
    }else{
        RCLCPP_INFO(node->get_logger(), "响应失败！");
    }
    // rclcpp::spin(node); // 服务通信不需要spin

    rclcpp::shutdown();
    return 0;
}