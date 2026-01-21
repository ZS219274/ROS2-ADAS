from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    启动 control 模块的 launch 文件
    
    启动 control_node 节点，订阅 /planning/local_trajectory 话题，
    使用 PID 控制算法控制车辆跟随规划轨迹，并广播 TF 变换。
    """
    
    # 路径设置
    # workspace/install/control/share/control 目录路径
    control_path = get_package_share_directory('control')
    
    # 启动控制节点
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[],
    )
    
    # 节点分组
    control = GroupAction(
      actions=[
        PushRosNamespace("control"),
        control_node,
      ]
    )
    
    return LaunchDescription(
      [
        control
      ]
    )

