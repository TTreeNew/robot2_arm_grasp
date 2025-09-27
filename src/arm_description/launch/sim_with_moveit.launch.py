import launch
import launch.event_handlers
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction

def generate_launch_description():
    """
    Launches both gazebo rviz2

    Includes
     * launch gazebo
     * robot_state_publisher
     * ros2_control_node + controller spawners
     * joint_state_broadcaster
     * arm_controller
    """
    robot_name="ros2_arm"
    packagepath = get_package_share_directory('arm_moveit_config')
    print(packagepath)    
    # urdf_pkg_path=get_package_share_directory('arm_moveit_config')  
    word_pkg_path=get_package_share_directory('arm_description')   ##  arm_description
    # default_model_path='/config/ar3.urdf.xacro'            ## /config/ar3.urdf.xacro 
    default_world_path='/world/world1.world'    ###允许传入world替换默认的相对路径
    
    # action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
    #     name='model',default_value=str(default_model_path),
    #     description='URDF的相对路径')

    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world',default_value=str(default_world_path),
        description='world的相对路径')

    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder("ar3", package_name="arm_moveit_config").to_moveit_configs()
    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        name="robot_state_publisher", 
        output="both", 
        parameters=[moveit_config.robot_description,
                    {'use_sim_time': True}, 
                    {'publish_frequency': 30.0}],
    )

    ##启动gazebo同时加载环境
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),'/launch/gazebo.launch.py']),
        launch_arguments=[('world',str(word_pkg_path)+default_world_path),('verbose','true'),]
    )

    ##生成加载机器人的节点，控制器在这里启动##
    spawn_entity_node=launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description','-entity', robot_name]
    )
    ##加载关节状态控制器，先启动后加载##
    load_joint_state_broadcaster = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster', '--set-state', 'active'],
        output='screen'
    )

    ##加载机械臂控制器##
    load_joint_trajectory_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'arm_controller', '--set-state', 'active'],
        output='screen'
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2", 
        output="log", 
        arguments=["-d",packagepath+'/config/moveit.rviz'],
        # arguments=["-d",'/home/tree/robot2_arm_grasp/src/arm_description/config/moveit.rviz'],
        
        parameters=[
        moveit_config.robot_description, 
        moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, moveit_config.planning_pipelines, 
        moveit_config.joint_limits, 
        {'use_sim_time': True}
    ],
    )    

    # 启动move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group", 
        executable="move_group", 
        output="screen", 
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': True}],
    )

    return LaunchDescription([
        # action_declare_arg_mode_path,
        action_declare_arg_world_path,
        launch_gazebo,
        robot_desc_node,
        spawn_entity_node, 

        launch.actions.RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_broadcaster],        
        )),        

        launch.actions.RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],        
        )),  
        move_group_node,

        # TimerAction(
        # period=7.0,
        # actions=[rviz_node]
        # )      
        rviz_node,
    ])
