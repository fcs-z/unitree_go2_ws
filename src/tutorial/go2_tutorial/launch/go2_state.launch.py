"""
启动驱动
加载自实现的位置获取节点
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


def generate_launch_description():
    
    go2_driver_pkg = get_package_share_directory('go2_driver')
    # go2_driver_launch = os.path.join(go2_driver_pkg, 'launch', 'go2_driver.launch.py')
    go2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go2_driver_pkg, 'launch', 'driver.launch.py'))
    )
    
    return LaunchDescription([
        go2_driver_launch,
        Node(
            package='go2_tutorial',
            executable='go2_state',
            parameters=[os.path.join(get_package_share_directory('go2_tutorial'), 'params', 'go2_state.yaml')],
        ),
        
    ])