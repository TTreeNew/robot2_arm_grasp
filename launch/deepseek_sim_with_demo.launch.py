# simulation_with_moveit.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. 启动您的仿真环境
    # 假设您的描述包名为 'arm_description'
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('arm_description'), 'launch', 'simulation.launch.py'])
        )
    )

    # 2. 启动MoveIt2，但进行关键配置：
    #    a) 使用仿真时间
    #    b) 禁用MoveIt2自带的robot_state_publisher（因为您的simulation.launch.py已经启动了一个）
    #    c) 可以选择是否启用RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('arm_moveit_config'), 'launch', 'moveit.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true', # 使用仿真时间
            'use_robot_state_publisher': 'false', # !!! 最关键的一行：禁用MoveIt2的robot_state_publisher !!!
            'launch_rviz': 'true'   # 如果您想在这里启动RViz就设为true
        }.items()
    )

    # 3. 可选但推荐：添加一个延迟或事件处理，确保仿真完全启动后再启动MoveIt2
    # 这里使用一个简单的延时器
    delayed_moveit_launch = TimerAction(
        period=5.0, # 延迟5秒，等待Gazebo和控制器稳定
        actions=[moveit_launch]
    )

    return LaunchDescription([
        simulation_launch,
        delayed_moveit_launch, # 使用延迟后的启动动作
    ])