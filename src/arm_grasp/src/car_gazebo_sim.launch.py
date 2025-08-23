import launch
import launch.event_handlers
import launch_ros
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name="carbot"
    urdf_pkg_path=get_package_share_directory('car_description_pkg')
    default_model_path='/urdf/fishbot/carbot.urdf.xacro'
    default_world_path='/world/custom_room.world'    ###允许传入model和world替换默认的相对路径

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='URDF的相对路径')

    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world',default_value=str(default_world_path),
        description='world的相对路径')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ',str(urdf_pkg_path),launch.substitutions.LaunchConfiguration('model')]),value_type=str)

    ##传入urdf参数，用来初始化robot_state_publisher的关节信息，并发布/robot_description话题
    robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description':robot_description}])

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),'/launch/gazebo.launch.py']),
        launch_arguments=[('world',str(urdf_pkg_path)+default_world_path),('verbose','true')]
    )
    ##生成加载机器人的节点，控制器在这里启动##
    spawn_entity_node=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description','-entity', robot_name, ]
    )
    ##加载关节状态控制器，先启动后加载##
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'carbot_joint_state_broadcaster', '--set-state', 'active'],
        output='screen'
    )

    ##load_carbot_effort_controller = launch.actions.ExecuteProcess(
    ##    cmd=['ros2', 'control', 'load_controller', 'carbot_effort_controller', '--set-state', 'active'],
    ##    output='screen'
    ##)

        ##加载两轮差速控制器##
    load_carbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'carbot_diff_drive_controller', '--set-state', 'active'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_arg_world_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,

        launch.actions.RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_controller],        
        )),

        launch.actions.RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_carbot_diff_drive_controller],        
        )),
        ##launch.actions.RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
        ##    target_action=load_joint_state_controller,
        ##    on_exit=[load_carbot_effort_controller],        
        ##)),        
    ])
