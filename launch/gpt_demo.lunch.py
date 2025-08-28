# moveit.launch.py
import os
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ---- 可按需修改的包名与默认路径 ----
    moveit_config_pkg = 'arm_moveit_config'     # 你的 MoveIt 配置包名
    robot_description_pkg = 'arm_description'   # 放 URDF(xacro) 的包名
    default_model_xacro = '/config/ar3.urdf.xacro'  # 相对于 robot_description_pkg 的 xacro 路径
    default_rviz_cfg = '/launch/moveit_rviz.rviz'   # 相对于 moveit_config_pkg 的 rviz 配置路径
    srdf_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'robot.srdf')  # 如果有 SRDF，可调整

    # ---- Launch 配置项 ----
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_xacro,
                                      description='urdf xacro path (relative to robot_description pkg)')
    rviz_arg = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_cfg,
                                     description='rviz config file (relative to moveit_config pkg)')
    use_rsp_arg = DeclareLaunchArgument(name='use_robot_state_publisher', default_value='false',
                                        description='是否在 MoveIt 启动 robot_state_publisher（如果仿真已经启动并发布 TF，设为 false）')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='是否使用仿真时间')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 生成 robot_description（使用 xacro 命令）
    robot_description_content = Command([
        'xacro ',
        os.path.join(get_package_share_directory(robot_description_pkg), ''), model
    ])

    # 若你有 SRDF/semantic 文件，可以加载为参数（这里尝试读取文件内容；若没有可删掉）
    srdf_fullpath = srdf_path
    robot_description_semantic = ''
    if os.path.exists(srdf_fullpath):
        with open(srdf_fullpath, 'r') as f:
            robot_description_semantic = f.read()

    # ---- 启动 move_group ----
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
            # 其他 MoveIt 配置文件（planning pipelines 等）可以在这里以 dict 或 yaml paths 加入
        ],
    )

    # ---- 可选的 robot_state_publisher（如果你不在 simulation 里启动它，可在 MoveIt 启动） ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_robot_state_publisher)  # 只有当参数为 true 时启动
    )

    # ---- 启动 RViz（用 MoveIt 的 rviz config） ----
    rviz_fullpath = os.path.join(get_package_share_directory(moveit_config_pkg), '')  # base
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(moveit_config_pkg), rviz_config.perform(None))],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    # add args
    ld.add_action(model_arg)
    ld.add_action(rviz_arg)
    ld.add_action(use_rsp_arg)
    ld.add_action(use_sim_time_arg)

    # add nodes
    ld.add_action(move_group_node)
    # robot_state_publisher_node 有条件地加入
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
