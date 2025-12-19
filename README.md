# unitree_go2_ws
```
mkdir -p unitree_go2_ws/src
cd src
mkdir base
mkdir helloworld
mkdir tutorial
```
## 一、helloworld
### 1、cpp
#### 1.1 功能包
```
cd unitree_go2_ws/src/helloworld
```
```
ros2 pkg create go2_helloworld --build-type ament_cmake --dependencies rclcpp unitree_api --node-name hello
```
#### 1.2 源文件
功能包go2_helloworld/src下创建hello.cpp(忽略)
添加依赖
```
/opt/ros/humble/include/**
~/unitree/unitree_ros2/cyclonedds_ws/install/unitree_api/include/**
```
```
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
```
#### 1.3 配置（忽略）
```
add_executable(hello src/hello.cpp)
ament_target_dependencies(
  hello
  "rclcpp"
  "unitree_api"
)

install(TARGETS 
  hello
  DESTINATION lib/${PROJECT_NAME}
)
```
#### 1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_helloworld
source install/setup.bash
ros2 run go2_helloworld hello
```
### 2、python
#### 2.1 功能包
```
cd unitree_go2_ws/src/helloworld
```
```
ros2 pkg create go2_helloworld_py --build-type ament_python --dependencies rclpy unitree_api --node-name hello
```
#### 2.2 源文件
功能包go2_helloworld/go2_helloworld下创建hello.py(忽略)
```
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
 
class HelloWorld(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.pub_ = self.create_publisher(Request, '/api/sport/request', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Hello World node has been started.")
    
    def timer_callback(self):
        request = Request()
        request.header.identity.api_id = 1016  # 打招呼的协议码
        self.pub_.publish(request)
        
def main():
    rclpy.init()
    node = HelloWorld("hello_world_py")
    rclpy.spin(node)
    rclpy.shutdown()
```
#### 2.3 配置（忽略）
```
'hello = go2_helloworld_py.hello:main'
```
#### 2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_helloworld_py
source install/setup.bash
ros2 run go2_helloworld_py hello
```
## 二、消息接口
### 1、状态获取
#### 1.1 高层状态
高层状态为机器人的速度、位置、足端位置等与运动相关的状态。高层状态的获取可通过订阅"lf/sportmodestate"或"sportmodestate" topic 实现，其中"lf"表示低频率
##### 1.1.1 查看消息类型
```
ros2 topic type /sportmodestate             高频
ros2 topic type /lf/sportmodestate          低频
ros2 topic type /mf/sportmodestate          中频
``` 
消息类型为: unitree_go/msg/SportModeState
```
ros2 topic list | grep -i sportmodestate
ro2 topic hz /lf/sportmodestate
```
##### 1.1.2 查看数据结构
```
ros2 interface show unitree_go/msg/SportModestate
```
接口定义的源文件为: /unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg
```
ros2 topic echo /sportmodestate --no-arr
```
##### 1.1.3 内置例程
读取高层状态的完整例程位于 unitree_ros2/example/src/read_motion_state.cpp
```
cd unitree_ros2/example
./install/unitree_ros2_example/bin/read_motion_state
```
高层状态信息的具体解释可参考：https://support.unitree.com/home/zh/developer/sports_services
#### 1.2 低层状态
低层状态为机器人的关节电机、电源信息等底层状态。通过订阅"lf/lowstate"或"lowstate" topic，可实现低层状态的获取
##### 1.2.1 查看消息类型
```
ros2 topic type /lowstate
ros2 topic type /lf/lowstate
```
消息类型为: unitree_go/msg/LowState
```
ros2 topic list | grep -i lowstate
ro2 topic hz /lf/lowstate
```
##### 1.2.2 查看数据结构
```
ros2 interface show unitree_go/msg/LowState
```
接口定义的源文件为: /unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/LowState.msg
##### 1.2.3 内置例程
读取低层状态的完整例程位于 unitree_ros2/example/src/read_low_state.cpp
```
cd unitree_ros2/example
./install/unitree_ros2_example/bin/read_low_state
```
低层状态信息的具体解释可参考: https://support.unitree.com/home/zh/developer/Basic_services
#### 1.3 遥控器状态
通过订阅"/wirelesscontroller" topic可获取遥控器的摇杆数值和按键键值
##### 1.3.1 查看消息类型
```
ros2 topic type /wirelesscontroller
```
消息类型为: unitree_go/msg/WirelessController
```
ros2 topic list | grep -i wirelesscontroller
```
##### 1.3.2 查看数据结构
```
ros2 interface show unitree_go/msg/WirelessController
```
接口定义的源文件为: /unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/WirelessController.msg
##### 1.3.3 内置例程
读取遥控器状态的完整例程序见: example/src/read_wireless_controller.cpp
```
cd unitree_ros2/example
./install/unitree_ros2_example/bin/read_wireless_controller
```
遥控器状态和遥控器键值的相关定义可参考：https://support.unitree.com/home/zh/developer/Get_remote_control_status
### 2、机器人控制
Go2机器人的运动指令是通过请求响应的方式实现的，通过订阅"/api/sport/request"，并发送运动unitree_api::msg::Request消息可以实现高层的运动控制
#### 2.1 运动控制
##### 2.1.1 查看消息类型
```
ros2 topic type /api/sport/request
``` 
消息类型为: unitree_api/msg/Request
##### 2.1.2 查看数据结构
```
ros2 interface show unitree_api/msg/Request
```
接口定义的源文件为: /unitree_ros2/cyclonedds_ws/src/unitree/unitree_api/msg/Request.msg
##### 2.1.3 内置例程
高层运动控制的完整例程位于 unitree_ros2/example/src/sport_mode_ctrl.cpp。暂无此文件
```
cd unitree_ros2/example
./install/unitree_ros2_example/bin/sport_mode_ctrl
```
关于SportClient运动控制接口的具体解释可参考：https://support.unitree.com/home/zh/developer/sports_services
#### 2.2 电机控制
通过订阅"/lowcmd" topic，并发送unitree_go::msg::LowCmd可以实现对电机的力矩、位置、和速度控制
##### 2.2.1 查看消息类型
```
ros2 topic type /lowcmd
```
消息类型为: unitree_go/msg/LowCmd
##### 2.2.2 查看数据结构
```
ros2 interface show unitree_go/msg/LowCmd
```
接口定义的源文件为: /unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/LowCmd
##### 2.2.3 内置例程
电机控制的完整例程位于 unitree_ros2/example/src/low_level_ctrl.cpp
```
cd unitree_ros2/example
./install/unitree_ros2_example/bin/low_level_ctrl
```
低层指令的具体解释可参考: https://support.unitree.com/home/zh/developer/Basic_services
## 三、基础功能包
### 1、键盘控制
#### 1.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_teleop_ctrl_keyboard --build-type ament_python --dependencies rclpy unitree_api --node-name go2_teleop_ctrl_keyboard
```
#### 1.2 源文件
功能包go2_teleop_ctrl_keyboard/go2_teleop_ctrl_keyboard下创建go2_teleop_ctrl_keyboard.py(忽略)
```

```
#### 1.3 配置(忽略)
```
'go2_teleop_ctrl_keyboard = go2_teleop_ctrl_keyboard.go2_teleop_ctrl_keyboard:main'
```
#### 1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_teleop_ctrl_keyboard
source install/setup.bash
ros2 run go2_teleop_ctrl_keyboard go2_teleop_ctrl_keyboard
```
### 2、ROS2 Twist消息桥接
#### 2.1 cpp
##### 2.1.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_twist_bridge --build-type ament_cmake --dependencies rclcpp geometry_msgs unitree_api --node-name twist_bridge
```
- 先将/unitree_ros2/example/src/include目录下的nlohmann目录，复制到当前功能包include目录下
- 在include目录下，新建sport_model.hpp文件，并输入如下内容：
```

```
##### 2.1.2 源文件
功能包go2_twist_bridge/src下创建twist_bridge.cpp(忽略)
```

```
##### 2.1.3 配置(忽略)
```
add_executable(twist_bridge src/twist_bridge.cpp)
ament_target_dependencies(
  twist_bridge
  "rclcpp"
  "geometry_msgs"
  "unitree_api"
)

install(TARGETS 
  twist_bridge
  DESTINATION lib/${PROJECT_NAME})
```
##### 2.1.4 编译运行
cd unitree_go2_ws
colcon build --packages-select go2_twist_bridge
source install/setup.bash
ros2 run go2_twist_bridge twist_bridge
#### 2.2 python
##### 2.2.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_twist_bridge_py --build-type ament_python --dependencies rclpy geometry_msgs unitree_api --node-name twist_bridge
```
- 功能包go2_twist_bridge_py/go2_twist_bridge_py下创建sport_model.py，并编辑文件，输入如下内容：
```

```
##### 2.2.2 源文件
功能包go2_twist_bridge_py/go2_twist_bridge_py下创建twist_bridge.py(忽略)
```

```
##### 2.2.3 配置(忽略)
```
'twist_bridge = go2_twist_bridge_py.twist_bridge:main'
```
##### 2.2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_twist_bridge_py
source install/setup.bash
ros2 run go2_twist_bridge_py twist_bridge 
```
### 3、机器人模型可视化
#### 3.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_description
```
- 访问宇树官网文档中心：https://support.unitree.com/home/zh/developer/ObtainSDK下载go2urdf。
解压缩后会获取一个GO2_URDF文件夹。然后将GO2_URDF下的dae、meshes、urdf三个文件夹复制到当前功能包下。
- 打开urdf/go2_description.urdf，删除其中的注释内容
#### 3.2 源文件
功能包go2_description下创建launch文件夹，launch中创建display.launch.py文件
```

```
##### 3.3 配置
```
install(DIRECTORY 
    launch urdf meshes dae
    DESTINATION share/${PROJECT_NAME}
)
```
```
<exec_depend>rviz2</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>ros2launch</exec_depend>
```
##### 3.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_description
source install/setup.bash
ros2 launch go2_description display.launch.py
```
ros2 run tf2_ros static_transform_publisher --frame-id radar --child-frame-id utlidar_lidar
```
source install/setup.bash
rviz2
```
### 4、ROS2 驱动包
#### 4.1 cpp
##### 4.1.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_driver --build-type ament_cmake --dependencies rclcpp unitree_go sensor_msgs tf2 tf2_ros geometry_msgs nav_msgs --node-name driver
```
功能包go2_driver下创建launch、rviz、params文件夹
##### 4.1.2 源文件
添加依赖
```
~/unitree/unitree_ros2/cyclonedds_ws/install/unitree_go/include/**
```
launch中创建driver.launch.py文件
```

```
功能包go2_driver/src下创建driver.cpp(忽略)
```

```
params中创建driver.yaml,启动driver节点,新终端输入ros2 param dump /driver,复制进driver.yaml,可删除qos_overrides下内容,/driver改为/**
```
/**:
  ros__parameters:
    base_frame: base
    odom_frame: odom
    publish_tf: true
    use_sim_time: false
```
##### 4.1.3 配置(部分忽略)
```
add_executable(driver src/driver.cpp)
ament_target_dependencies(
  driver
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch params rviz
  DESTINATION share/${PROJECT_NAME}/
)
```
##### 4.1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_driver
source install/setup.bash
ros2 launch go2_driver driver.launch.py
```
#### 4.2 python
##### 4.2.1 功能包
```
cd unitree_go2_ws/src/base
```
```
ros2 pkg create go2_driver_py --build-type ament_python --dependencies rclpy unitree_go sensor_msgs tf2_ros geometry_msgs nav_msgs --node-name driver
```
功能包go2_driver_py下创建launch、rviz、params文件夹
##### 4.2.2 源文件
launch中创建driver.launch.py文件
```

```
功能包go2_driver_py/go2_driver_py下创建driver.py(忽略)
```

```
params中创建driver.yaml,启动driver_py节点,新终端输入ros2 param dump /driver_py,复制进driver.yaml,/driver_py改为/**
```
/**:
  ros__parameters:
    base_frame: base
    odom_frame: odom
    publish_tf: true
    use_sim_time: false
```
##### 4.2.3 配置
```
from glob import glob
('share/' + package_name + '/launch', glob('launch/*.launch.py')),  
('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
('share/' + package_name + '/params', glob('params/*.yaml')),
'driver = go2_driver_py.driver:main'
```
##### 4.2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_driver_py
source install/setup.bash
ros2 launch go2_driver_py driver.launch.py
```
## 四、通信例程
### 1、go2巡航与位置获取
#### 1.1 cpp
##### 1.1.1 功能包
```
ros2 pkg create go2_tutorial --build-type ament_cmake --dependencies rclcpp unitree_go unitree_api --node-name go2_ctrl
```
- 先将/unitree_ros2/example/src/include目录下的nlohmann目录，复制到当前功能包include目录下
- 在include目录下，新建sport_model.hpp文件，并输入如下内容：
```

```
新建launch、params文件夹
##### 1.1.2 源文件
go2_ctrl.cpp:
```

```
go2_ctrl.launch.py:
```

```
go2_state.cpp:
```

```
go2_state.launch.py:
```

```
go2_state.yaml
```
/**:
  ros__parameters:
    distance: 0.5
    use_sim_time: false
```
##### 1.1.3 配置
```
find_package(nav_msgs REQUIRED)

add_executable(go2_state src/go2_state.cpp)
ament_target_dependencies(
  go2_state
  "rclcpp"
  "nav_msgs"
)

install(TARGETS go2_ctrl go2_state
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch params
DESTINATION share/${PROJECT_NAME}
)
```
```
<depend>nav_msgs</depend>
```
##### 1.1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial
source install/setup.bash
ros2 launch go2_tutorial go2_ctrl.launch.py
ros2 launch go2_tutorial go2_state.launch.py
```
#### 1.2 python
##### 1.2.1 功能包
```
ros2 pkg create go2_tutorial_py --build-type ament_python --dependencies rclpy unitree_go unitree_api --node-name go2_ctrl
```
- 功能包go2_twist_bridge_py/go2_twist_bridge_py下创建sport_model.py，并编辑文件，输入如下内容：
```

```
新建launch、params文件夹
##### 1.2.2 源文件
go2_ctrl.py:
```

```
go2_ctrl.launch.py:
```

```
go2_state.py:
```

```
go2_state.launch.py:
```

```
go2_state.yaml
```
/**:
  ros__parameters:
    distance: 0.5
    use_sim_time: false
```
##### 1.2.3 配置
```
from glob import glob
('share/' + package_name + "/launch", glob("launch/*.launch.py")),
('share/' + package_name + "/params", glob("params/*.yaml")),
```
```
'go2_state = go2_tutorial_py.go2_state:main'
```
```
<depend>nav_msgs</depend>
```
##### 1.2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial_py
source install/setup.bash
ros2 launch go2_tutorial_py go2_ctrl.launch.py
ros2 launch go2_tutorial_py go2_state.launch.py
```

### 2、go2巡航启停
#### 2.1 cpp
##### 2.1.1 功能包
```
ros2 pkg create go2_tutorial_inter
```
##### 2.1.2 源文件
功能包go2_tutorial_inter:
- 新建srv文件夹，srv文件夹下新建Cruising.srv文件，文件中输入如下内容：
```
int32 flag
---
geometry_msgs/Point point
```
功能包go2_tutorial:
- src/新建go2_cruising_service.cpp, ~/unitree/unitree_go2_ws/install/go2_tutorial_inter/include/**
```

```
- src/新建go2_cruising_client.cpp
```

```
- launch/新建go2_cruising.launch.py
```

```
- go2_cruising_service.yaml
ros2 param dump /go2_crusing_service
```
/**:
  ros__parameters:
    use_sim_time: false
    x: 0.1
    y: 0.0
    z: 0.5
```
##### 2.1.3 配置
功能包go2_tutorial_inter:
```
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Cruising.srv"
  DEPENDENCIES geometry_msgs
)
```
```
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
功能包go2_tutorial：
```
find_package(go2_tutorial_inter REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(go2_cruising_service src/go2_cruising_service.cpp)
target_include_directories(go2_cruising_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  go2_cruising_service
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
)
add_executable(go2_cruising_client src/go2_cruising_client.cpp)
ament_target_dependencies(
  go2_cruising_client
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
)

install(TARGETS 
  go2_ctrl 
  go2_state 
  go2_cruising_service 
  go2_cruising_client
DESTINATION lib/${PROJECT_NAME}
)
```
```
<depend>go2_tutorial_inter</depend>
<depend>geometry_msgs</depend>
```
##### 2.1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial_inter
colcon build --packages-select go2_tutorial

source install/setup.bash
ros2 interface show go2_tutorial_inter/srv/Cruising
ros2 run go2_tutorial go2_cruising_service
ros2 service call /cruising go2_tutorial_inter/srv/Cruising "{flag: 1}"

source install/setup.bash
ros2 launch go2_tutorial go2_cruising.launch.py
ros2 run go2_tutorial go2_cruising_client 1
```
#### 2.2 python
##### 2.2.1 功能包
```

```
##### 2.2.2 源文件
功能包go2_tutorial_inter:
- 新建srv文件夹，srv文件夹下新建Cruising.srv文件，文件中输入如下内容：
```
int32 flag
---
geometry_msgs/Point point
```
功能包go2_tutorial_py:
- src/新建go2_cruising_service.py
```

```
- src/新建go2_cruising_client.py
```

```
- launch/新建go2_cruising.launch.py
```

```
- go2_cruising_service.yaml
ros2 param dump /go2_crusing_service_py
```
/**:
  ros__parameters:
    use_sim_time: false
    x: 0.1
    y: 0.0
    z: 0.5
```
##### 2.2.3 配置
```
'go2_cruising_service = go2_tutorial_py.go2_cruising_service:main',
'go2_cruising_client = go2_tutorial_py.go2_cruising_client:main',
```
```
<depend>go2_tutorial_inter</depend>
<depend>geometry_msgs</depend>
```
##### 2.2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial_inter
colcon build --packages-select go2_tutorial_py

source install/setup.bash
ros2 launch go2_tutorial_py go2_cruising.launch.py
ros2 run go2_tutorial_py go2_cruising_client 1
```

### 3、go2导航
#### 3.1 cpp
##### 3.1.1 功能包
```

```
##### 3.1.2 源文件
功能包go2_tutorial_inter:
- 新建action文件夹，action文件夹下新建Nav.action文件，文件中输入如下内容：
```
# 请求数据（前进距离）
float32 goal
---
# 最终响应（坐标）
geometry_msgs/Point point
---
# 连续反馈（剩余距离）
float32 distance
```
功能包go2_tutorial:
- src/新建go2_nav_server.cpp, ~/unitree/unitree_go2_ws/install/go2_tutorial_inter/include/**
```

```
- src/新建go2_nav_client.cpp
```

```
- launch/新建go2_nav.launch.py
```

```
- go2_nav_server.yaml
ros2 param dump /go2_nav_server
```
/**:
  ros__parameters:
    error: 0.2
    use_sim_time: false
    x: 0.1
```
##### 3.1.3 配置
功能包go2_tutorial_inter:
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Cruising.srv"
  "action/Nav.action"
  DEPENDENCIES geometry_msgs
)
```
功能包go2_tutorial:
```
find_package(rclcpp_action REQUIRED)

add_executable(go2_nav_server src/go2_nav_server.cpp)
target_include_directories(go2_nav_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  go2_nav_server
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
  "rclcpp_action"
)
add_executable(go2_nav_client src/go2_nav_client.cpp)
ament_target_dependencies(
  go2_nav_client
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "nav_msgs"
  "go2_tutorial_inter"
  "geometry_msgs"
  "rclcpp_action"
)

install(TARGETS 
  go2_ctrl 
  go2_state 
  go2_cruising_service 
  go2_cruising_client
  go2_nav_server
  go2_nav_client
DESTINATION lib/${PROJECT_NAME}
)
```
```
<depend>rclcpp_action</depend>
```
##### 3.1.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial_inter
colcon build --packages-select go2_tutorial

source install/setup.bash
ros2 interface show go2_tutorial_inter/srv/Cruising
ros2 launch go2_tutorial go2_nav.launch.py
ros2 action send_goal /nav go2_tutorial_inter/action/Nav "{goal: 1}" -f

source install/setup.bash
ros2 launch go2_tutorial go2_nav.launch.py
ros2 run go2_tutorial go2_nav_client 0.3
```
#### 3.2 python
##### 3.2.1 功能包
```

```
##### 3.2.2 源文件
功能包go2_tutorial_inter:
- 新建action文件夹，action文件夹下新建Nav.action文件，文件中输入如下内容：
```
# 请求数据（前进距离）
float32 goal
---
# 最终响应（坐标）
geometry_msgs/Point point
---
# 连续反馈（剩余距离）
float32 distance
```
功能包go2_tutorial_py:
- src/新建go2_nav_server.py
```

```
- src/新建go2_nav_client.py
```

```
- launch/新建go2_nav.launch.py
```

```
- go2_nav_server.yaml
ros2 param dump /go2_nav_server_py
```
/**:
  ros__parameters:
    error: 0.2
    use_sim_time: false
    x: 0.1
```
##### 3.2.3 配置
```
'go2_nav_server = go2_tutorial_py.go2_nav_server:main',
'go2_nav_client = go2_tutorial_py.go2_nav_client:main',
```
##### 3.2.4 编译运行
```
cd unitree_go2_ws
colcon build --packages-select go2_tutorial_inter
colcon build --packages-select go2_tutorial_py

source install/setup.bash
ros2 launch go2_tutorial_py go2_nav.launch.py
ros2 run go2_tutorial_py go2_nav_client 0.3
```
