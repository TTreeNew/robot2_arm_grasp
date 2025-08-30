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

    path=__file__.split('motion_api/test3')[0]+'config/moveit_cpp.yaml'

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
    arm_group = robot.get_planning_component("arm")
    logger.info("MoveItPy instance created")
    print(robot)


    # 设置机械臂起始位置为当前状态位置
    arm_group.set_start_state_to_current_state()

    # set pose goal with PoseStamped message 
    from geometry_msgs.msg import PoseStamped 
    
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link" #表示这个位姿是在哪个参考坐标系下定义的。通常是 机器人底座 或世界坐标系。
    pose_goal.pose.orientation.x = 0.71
    pose_goal.pose.orientation.y = 0.71

    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = 0.68
    pose_goal.pose.position.z =0.17  
    #设置目标位置为指定的直角坐标系位置
    arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="gripper_base_link") #表示你希望最终到达目标位姿的末端执行器链接名称。
    
    #进行路径规划并执行
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)