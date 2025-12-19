/*
导航服务端
需求:
    1.处理客户端请求
        提交的请求为浮点数（前进距离）
        如果大于0，那么控制机器人往前运动，否则认为数据不合法
    2.处理客户端取消请求
        当客户端发送取消请求，那么需要让机器人停止运动
    3.产生连续反馈和最终响应
        根据机器人当前坐标，结合起点坐标，结合前进距离计算剩余距离并周期性反馈
        当机器人到达目标点，响应机器人坐标
    实现:
        1.创建action服务端，执行相关操作；
        2.需要向πgo2ctrl注入参数控制机器人运动或停止；
        3.订阅机器人里程计获取坐标。
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <go2_tutorial_inter/action/nav.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sport_model.hpp>

using namespace rclcpp_action;
using namespace std::placeholders;
using namespace std::chrono_literals;
 
class Go2NavServer: public rclcpp::Node{
public:
    Go2NavServer(const std::string &node_name):Node(node_name){
        RCLCPP_INFO(this->get_logger(),"Go2NavServer 创建");
        nav_server_ = rclcpp_action::create_server<go2_tutorial_inter::action::Nav>(
            this,"nav",
            std::bind(&Go2NavServer::goal_cb,this,_1,_2),
            std::bind(&Go2NavServer::cancel_cb,this,_1),
            std::bind(&Go2NavServer::accepted_cb,this,_1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,
            std::bind(&Go2NavServer::odom_cb,this,_1));

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

        // 设置巡航参数
        this->declare_parameter("x", 0.1);
        this->declare_parameter("error", 0.2);
    }
private:

    GoalResponse goal_cb(const GoalUUID &uuid, std::shared_ptr<const go2_tutorial_inter::action::Nav::Goal> goal){
        (void) uuid;
        // 获取前进距离
        float goal_dis = goal->goal;
        // 判断合法性
        // 合法，控制go2向前运动，否则，拒绝任务
        if(goal_dis > 0){
            RCLCPP_INFO(this->get_logger(),"前进%.2f米",goal_dis);
            // 让go2开始运动
            param_client_->set_parameters({
                rclcpp::Parameter("sport_api_id", ROBOT_SPORT_API_ID_MOVE),// go2_ctrl节点的sport_api_id参数
                // rclcpp::Parameter("x", 0.1), 
                this->get_parameter("x"), // 本节点的参数
            });
            return GoalResponse::ACCEPT_AND_EXECUTE;
        }else{
            RCLCPP_ERROR(this->get_logger(),"前进%.2f米,距离小于0,是非法数据!",goal_dis);
            return GoalResponse::REJECT;
        }
    };
    // 处理客户端提交的取消请求
    CancelResponse cancel_cb(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        (void) server_goal_handle;
        RCLCPP_INFO(this->get_logger(),"取消请求");
        // 让机械狗停止运动
        stop_move();
        // 无条件接受取消请求
        return CancelResponse::ACCEPT;
    };
    void stop_move(){
        param_client_->set_parameters({
            rclcpp::Parameter("sport_api_id", ROBOT_SPORT_API_ID_STOPMOVE),
            rclcpp::Parameter("x", 0.0),
            rclcpp::Parameter("y", 0.0),
            rclcpp::Parameter("z", 0.0)
        });
    }
    void accepted_cb(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        RCLCPP_INFO(this->get_logger(),"开始执行任务");
        std::thread(std::bind(&Go2NavServer::execute,this,_1),server_goal_handle).detach();
    };
    void execute(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){ 
        auto feedback = std::make_shared<go2_tutorial_inter::action::Nav::Feedback>();
        auto result = std::make_shared<go2_tutorial_inter::action::Nav::Result>();
        // 1. 生成连续反馈
        rclcpp::Rate rate(1.0);
        while(rclcpp::ok()){ 
            // 生成剩余距离 --- 获取当前坐标点和起点 --- 剩余距离 = 目标距离 - 已行进距离
            // 已行进距离
            auto dis_x = current_point.x - start_point.x;
            auto dis_y = current_point.y - start_point.y;
            auto dis = sqrt(pow(dis_x,2) + pow(dis_y,2));
            // 剩余距离 = 目标距离 - 已行进距离
            auto distance = server_goal_handle->get_goal()->goal - dis;
            // 发布剩余距离
            feedback->distance = distance;
            server_goal_handle->publish_feedback(feedback);
            // 循环退出条件
            // 任务被取消（客户端提交取消任务请求）
            if(server_goal_handle->is_canceling()){
                RCLCPP_INFO(this->get_logger(),"任务被取消");
                // 让机械狗停止运动
                stop_move();
                // 响应取消任务
                result->point = current_point;
                server_goal_handle->canceled(result);
                return;
            }
            // 任务已完成（到达目标点附近）
            if(distance <= this->get_parameter("error").as_double()){
                RCLCPP_INFO(this->get_logger(),"任务完成");
                stop_move();
                break;
            }
            rate.sleep();
        }
        // 2. 生成最终结果
        if(rclcpp::ok()){
            result->point = current_point;
            server_goal_handle->succeed(result);
        }
    }
    // 订阅里程计消息
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        current_point = odom->pose.pose.position;
    }

    rclcpp_action::Server<go2_tutorial_inter::action::Nav>::SharedPtr nav_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Point current_point, start_point;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    
};
 
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2NavServer>("go2_nav_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}