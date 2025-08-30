import time
import rclpy
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy, 
    MultiPipelinePlanRequestParameters, 
)
from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(
    robot,
    planning_component, 
    logger, 
    single_plan_parameters=None, 
    multi_plan_parameters=None, 
    sleep_time=0.0, 
):
    """Helper function to plan and execute a motion."""

    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

if __name__ == "__main__":
    # MoveItPy Setup
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    path=__file__.split('motion_api/test2')[0]+'config/moveit_cpp.yaml'

    print(f'moveit cpp config path is :(path)')
    
    moveit_config =(
        MoveItConfigsBuilder(
        robot_name="arm", package_name="robot_arm_config"
        )
        .moveit_cpp(path)
        .to_moveit_configs()
        )

    params=moveit_config.to_dict()#节点moveitpy的参数

    # instantiate MoveItPy instance and get planning component 
    robot = MoveItPy(node_name="moveit_py", config_dict=params)
    logger.info("MoveItPy instance created")
    print(robot)


    #########方法一：随机设置关节角位置
    # 创建RobotState对象
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)

    # 生成一个随机的机器人状态，并将其设置为目标状态
    robot_state.set_to_random_positions()
    arm_group = robot.get_planning_component("arm")
    logger.info("Set goal state to the initialized robot state")
    arm_group.set_goal_state(robot_state=robot_state)

    # 设置机械臂起始位置为当前状态位置
    arm_group.set_start_state_to_current_state()
    #进行路径规划并执行
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)


    ##########方法二：通过关节角设置位置
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    # 指定关节位置，并将其设置为目标状态
    robot_state.set_joint_group_positions('arm', [0, -0.4, -0.79, -0.79, -1.10, 1.55])
    arm_group.set_goal_state(robot_state=robot_state)

    #设置机械臂起始位置为当前状态位置
    arm_group.set_start_state_to_current_state()
    #进行路径规划并执行
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)

    # get_robot_model() 获取机械臂的几何和运动学信息，包括：
    # 关节数量、关节类型（旋转/平移）
    # 各个 link 的名称和父子关系
    # 运动学约束、关节限位
    # SRDF 信息（规划组、默认姿态等）