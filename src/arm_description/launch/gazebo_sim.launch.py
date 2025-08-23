import launch
import launch.event_handlers
import launch_ros
import launch_ros.parameter_descriptions
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Launches gazebo with a robot model and environment

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * ros2_control_node + controller spawners
    """
    robot_name="ros2_arm"
    urdf_pkg_path=get_package_share_directory('arm_moveit_config')  ###允许传入model和world替换默认的相对路径
    word_pkg_path=get_package_share_directory('arm_description')   ## car_description_pkg arm_description
    default_model_path='/config/ar3.urdf.xacro'            ## /config/ar3.urdf.xacro  /urdf/fishbot/carbot.urdf.xacro
    default_world_path='/world/world1.world'    ###允许传入model和world替换默认的相对路径
    
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='URDF的相对路径')

    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world',default_value=str(default_world_path),
        description='world的相对路径')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ',str(urdf_pkg_path),launch.substitutions.LaunchConfiguration('model')]),value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description':robot_description}])


    ##启动gazebo同时加载环境
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),'/launch/gazebo.launch.py']),
        launch_arguments=[('world',str(word_pkg_path)+default_world_path),('verbose','true')]
    )

    ##生成加载机器人的节点，控制器在这里启动##
    spawn_entity_node=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description','-entity', robot_name]
    )
    ##加载关节状态控制器，先启动后加载##
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'arm_joint_state_broadcaster', '--set-state', 'active'],
        output='screen'
    )

            ##加载机械臂控制器##
    load_joint_trajectory_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'arm_joint_trajectory_controller', '--set-state', 'active'],
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
            on_exit=[load_joint_trajectory_controller],        
        )),    
    ])