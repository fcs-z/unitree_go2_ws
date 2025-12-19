"""
launch文件需要集成功能
    1.机器人模型可视化；
    2.速度消息桥接；
    3.里程计消息发布、广播里程计坐标变换、发布关节状态信息。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory

import os
from launch.conditions import IfCondition


def generate_launch_description():
    
    go2_desc_pkg = get_package_share_directory('go2_description')
    go2_driver_pkg = get_package_share_directory('go2_driver_py')
    
    # rviz2 启动开关
    # ros2 launch qo2 driver driver.launch.py use rviz:=false
    use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='是否启动rviz2进行可视化'
    )
    
    return LaunchDescription([
        use_rviz,
        # 机器人模型可视化launch文件包含
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(go2_desc_pkg, 'launch', 'display.launch.py')
            ),
            launch_arguments=[('use_joint_state_publisher', 'false')],  # 关节状态由driver发布，不使用默认的joint_state_publisher
        ),
        # 包含rviz2节点
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(go2_driver_pkg, 'rviz', 'display.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
        # 雷达坐标系映射
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id","radar",
                       "--child-frame-id","utlidar_lidar"]
        ),
        # 速度消息桥接节点
        Node(
            package="go2_twist_bridge_py",
            executable="twist_bridge",
        ),
        # 里程计消息发布、广播里程计坐标变换、发布关节状态信息节点
        Node(
            package="go2_driver_py",
            executable="driver",
            parameters=[os.path.join(go2_driver_pkg, 'params', 'driver.yaml')],
        ),
        # 
    ])