import time
import rclpy
from rclpy.logging import get_logger
# moveit python library
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
    sleep_time= 0.0, 
):
    """Helper function to plan and execute  motion."""
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


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    robot = MoveItPy(node_name="moveit_py")
    arm_group = robot.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    # set plan start state using predefined state
    arm_group.set_start_state(configuration_name="stand")

    # set pose goal using predefined state
    arm_group.set_goal_state(configuration_name="ready")
    
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)
    while True: #循环移动
        #设置从起点移到终点
        arm_group.set_start_state(configuration_name="ready")
        # set pose goal using predefined state
        arm_group.set_goal_state(configuration_name="stand")
        plan_and_execute(robot, arm_group, logger, sleep_time=3.0)

        #设置从终点移到起点
        arm_group.set_start_state(configuration_name="stand")
        # set pose goal using predefined state
        arm_group.set_goal_state(configuration_name="ready")
        plan_and_execute(robot, arm_group, logger, sleep_time=3.0)