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

    path=__file__.split('motion_api/test')[0]+'config/moveit_cpp.yaml'

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