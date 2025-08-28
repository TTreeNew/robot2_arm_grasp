import time
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(
    robot, 
    planning_component, 
    logger, 
    single_plan_parameters=None, multi_plan_parameters=None, sleep time=3.0, 
):
    """Helper function to plan and execute a mdeion."""
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
    robot = MoveItPy(node_namem="moveit_py")
    arm_group = robot.get_planning_component("arm")
    hand_group = robot.get_planning_component('hand')
    logger.info("MoveItPy instance created")

##[-1.57079622 -0.12612029 -0.62965342 -1.83698228 -0.54883666 1.5707967 ]


    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state("stand")
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=10.0)


    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('arm', [-1.57079622, -0.12612029,-0.62965342 , -1.83698228, -0.54883666, 1.5707967 ])
    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state(robot_state=robot_state)
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)


    # set plan start state using predefined state 
    hand_group.set_start_state_to_current_state()
    # set pose goal using predefined state
    hand_group.set_goal_state("close")
    # plan to goal
    plan_and_execute(robot, hand_group, logger, sleep_time=3.0)

    robot_state.set_joint_group_positions('arm', [0, -0, -0.488692, -0.558505, -1.134464, 1.55])
    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state(robot_state=robot_state)
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=1.0)


    robot_state.set_joint_group_positions('arm', [0, -0.35, -0.7, -0.7, -1.10, 1.55])
    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state(robot_state=robot_state)
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)


    # set plan start state using predefined state 
    hand_group.set_start_state_to_current_state()
    # set pose goal using predefined state
    hand_group.set_goal_state("open")

    # plan to goal
    plan_and_execute(robot, hand_group, Iogger, sleep_time=3.9)
    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state("stand")
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)


    logger.info('运动完成')
    rclpy.shutdown()

if __name__ == "__main__":
    main()