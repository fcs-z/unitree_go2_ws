# from launch import LaunchDescription
# from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    
    # 获取功能包路径
    go2_description_pkg = get_package_share_directory("go2_description")
    # 声明一个布尔参数，用于控制是否启动joint_state_publisher
    use_joint_state_pulisher = DeclareLaunchArgument(
        name="use_joint_state_pulisher",
        default_value="True",
        description="Flag to enable joint_state_publisher"
    )
    model = DeclareLaunchArgument(
        name="urdf_path",
        default_value=os.path.join(go2_description_pkg, "urdf", "go2_description.urdf"),
    )
    
    # 使用xacro读取urdf文件中的内容
    # robot_desc = ParameterValue(Command(["xacro ",os.path.join(go2_description_pkg, "urdf", "go2_description.urdf")]))
    robot_desc = ParameterValue(Command(["xacro ",LaunchConfiguration("urdf_path")]))
    
    # robot_state_publisher --- 加载机器人 urdf 文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[{"robot_description":robot_desc}]  
    )
    
    # joint_state_publisher --- 发布关节状态
    # 以后更合理的方式是由程序动态获取关节信息并发布
    # 这个节点的启动应该灵活启动，有条件
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_pulisher"))
    )

    return LaunchDescription([
        model,
        use_joint_state_pulisher,
        robot_state_publisher,
        joint_state_publisher,
       
    ])

