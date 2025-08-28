from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    packagepath = get_package_share_directory('arm_moveit_config')
    print(packagepath)

    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder("ar3", package_name="arm_moveit_config").to_moveit_configs()
    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        name="robot_state_publisher", 
        output="both", 
        parameters=[moveit_config.robot_description], 
    )
    # Static TF 发布机械臂虚拟关节坐标系
    static_tf_node = Node(
        package="tf2_ros", 
        executable="static_transform_publisher", 
        name="static_transform_publisher",
        output="log", 
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2", 
        output="log", 
        arguments=["-d",packagepath+'/config/moveit.rviz'],
        parameters=[
        moveit_config.robot_description, 
        moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, moveit_config.planning_pipelines, 
        moveit_config.joint_limits, 
    ],
    )

    #ros2_controller manger节点
    ros2_control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description,
                    packagepath+'/config/ros2_controllers.yaml'],
        output="both", 
    )

    #启动关节状态发布器，arm组控制器，夹抓控制器
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=[
        "joint_state_broadcaster", "arm_controller"
        #, "hand_controller"
        ],
    )

    # 启动move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group", 
        executable="move_group", 
        output="screen", 
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        robot_desc_node, 
        static_tf_node, 
        ros2_control_node, 
        controller_spawner_node , 
        move_group_node,
        rviz_node,
    ])