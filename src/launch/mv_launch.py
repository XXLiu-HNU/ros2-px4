from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # 添加ros2 run指令到launch文件
    node1_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'px4_ros_com', 'uva1_takeoff'],
        output='screen'
    )

    node2_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'px4_ros_com', 'uva2_takeoff'],
        output='screen'
    )
    node3_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'px4_ros_com', 'uva3_takeoff'],
        output='screen'
    )

    node4_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'px4_ros_com', 'uva4_takeoff'],
        output='screen'
    )

    ld.add_action(node1_cmd)
    ld.add_action(node2_cmd)
    ld.add_action(node3_cmd)
    ld.add_action(node4_cmd)
    return ld
