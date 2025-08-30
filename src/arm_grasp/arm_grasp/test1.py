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
    planning_component,  #翻译：规划组件
    logger, 
    single_plan_parameters=None, 
    multi_plan_parameters=None, 
    sleep_time= 0.0, 
):
    """Helper function to plan and execute a motion."""

    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    #解释：planning_component.plan()的签名是：plan(single_plan_parameters=None, multi_plan_parameters=None)，
    #所以需要关键字参数，而不是直接传一个位置参数
    #也就是说需要写成plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    #而不是plan_result = planning_component.plan(multi_plan_parameters）
        
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

    path=__file__.split('motion_api/test1')[0]+'config/moveit_cpp.yaml'

    print(f'moveit cpp config path is :(path)')
    
    moveit_config =(
        MoveItConfigsBuilder(
        robot_name="arm", package_name="robot_arm_config"
        )
        .moveit_cpp(path)
        .to_moveit_configs()
        )

    params=moveit_config.to_dict()#节点moveitpy的参数 dict:字典

    # instantiate MoveItPy instance and get planning component 
    robot = MoveItPy(node_name="moveit_py", config_dict=params)
    arm_group = robot.get_planning_component("arm")
    logger.info("MoveItPy instance created")
    print(robot)


    # 设置起始为预定义的stand位置
    arm_group.set_start_state(configuration_name="stand")
    # 设置终点为预定义的ready位置
    arm_group.set_goal_state(configuration_name="ready")
    # 进行路径规划并执行
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)