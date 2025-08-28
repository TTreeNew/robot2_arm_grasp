# simulation_with_moveit.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 默认子 Launch 所在包与文件名（按需修改）
    simulation_launch_pkg = 'arm_moveit_config'   # 放 simulation.launch.py 的包名（如果你把 simulation.launch.py 放在别的包，改这里）
    simulation_launch_file = 'simulation.launch.py'  # 你的仿真文件名
    moveit_launch_pkg = 'arm_moveit_config'       # 放 moveit.launch.py 的包名
    moveit_launch_file = 'moveit.launch.py'      # 上面生成的 moveit 文件名

    # 把一些参数做成顶层可配置项
    model_arg = DeclareLaunchArgument(name='model', default_value='/config/ar3.urdf.xacro',
                                      description='URDF xacro 文件，相对于 robot_description 包')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='是否使用仿真时间')
    # 是否让 moveit 启动 robot_state_publisher（通常仿真会已经发布 TF，所以默认为 false）
    use_rsp_arg = DeclareLaunchArgument(name='use_robot_state_publisher', default_value='false',
                                        description='是否在 moveit 部分启动 robot_state_publisher')

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_publisher = LaunchConfiguration('use_robot_state_publisher')

    # Include 仿真 launch（把 model、use_sim_time 等参数传给它，如果你的 simulation.launch.py 接受这些参数）
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(simulation_launch_pkg), 'launch', simulation_launch_file)
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time
            # 如有其它参数（比如 controller 名称、world 路径等），在这里继续转发
        }.items()
    )

    # Include MoveIt launch（把相同的 model 和 use_sim_time 传下去，并决定是否启动 robot_state_publisher）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(moveit_launch_pkg), 'launch', moveit_launch_file)
        ),
        launch_arguments={
            'model': model,
            'use_sim_time': use_sim_time,
            'use_robot_state_publisher': use_robot_state_publisher
        }.items()
    )

    ld = LaunchDescription()

    # add args
    ld.add_action(model_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_rsp_arg)

    # 顺序：先仿真，再 MoveIt（具体依赖你 simulation.launch.py 的事件/顺序机制）
    ld.add_action(simulation_launch)
    ld.add_action(moveit_launch)

    return ld
