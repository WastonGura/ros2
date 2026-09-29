import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动乌龟仿真器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        # 2. 启动补能系统节点 (控制 turtle2)
        Node(
            package='sheep_patrol',
            executable='charger_node',
            name='charger_node',
            output='screen',
            parameters=[{
                'power_charge': 100.0,
                'overheat_temp': 60.0,
                'cooling_resume_temp': 42.0,
                'target_charge_time': 15.0,
                'filter_window_size': 15
            }]
        ),
        # 3. 启动巡检机器人节点 (控制 turtle1)
        Node(
            package='sheep_patrol',
            executable='patrol_node',
            name='patrol_node',
            output='screen'
        )
    ])
