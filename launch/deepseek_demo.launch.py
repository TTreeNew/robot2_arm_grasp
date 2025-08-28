# moveit.launch.py
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. 声明可配置的参数
    # 是否启动RViz（如果在总控文件中，我们可能会禁用RViz而使用自己的配置）
    declare_launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    # 是否启动robot_state_publisher（关键！在总控中我们将把它设为false）
    declare_use_robot_state_pub_arg = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='true',
        description='Whether to start the robot_state_publisher node. Set to false if it is already provided elsewhere (e.g., by your simulation.launch.py).'
    )
    # 是否使用仿真时间
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. 加载MoveIt配置
    # 假设您的MoveIt2配置包名为 'arm_moveit_config'
    robot_description_content = launch.substitutions.Command(
        ['xacro ', PathJoinSubstitution([FindPackageShare('arm_moveit_config'), 'config', 'ros2_arm.srdf.xacro'])]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_content = launch.substitutions.Command(
        ['xacro ', PathJoinSubstitution([FindPackageShare('arm_moveit_config'), 'config', 'ros2_arm.srdf.xacro'])]
    )
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # 3. 启动move_group节点 (MoveIt2的核心)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
            # 其他MoveIt2配置通常会从一个YAML文件中加载，例如：
            # PathJoinSubstitution([FindPackageShare('arm_moveit_config'), 'config', 'moveit_controllers.yaml'])
        ],
    )

    # 4. 有条件地启动robot_state_publisher
    # ！！！这是避免冲突的关键！！！
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # 只有 when 'use_robot_state_publisher' 参数为 true 时才启动
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_robot_state_publisher'))
    )

    # 5. 有条件地启动RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('arm_moveit_config'), 'config', 'moveit.rviz'])],
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        # 只有 when 'launch_rviz' 参数为 true 时才启动
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        declare_launch_rviz_arg,
        declare_use_robot_state_pub_arg,
        declare_use_sim_time_arg,
        
        move_group_node,
        robot_state_publisher_node,
        rviz_node,
    ])