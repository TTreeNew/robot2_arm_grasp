#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

void plan_and_execute(
    moveit::planning_interface::MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    const geometry_msgs::msg::PoseStamped &pose_target,
    double sleep_time = 0.0
)
{

    move_group.setPoseTarget(pose_target);

    // 规划轨迹
    RCLCPP_INFO(logger, "Planning target trajectory...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(logger, "Executing plan...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // 等待
    if (sleep_time > 0.0)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
}

void plan_and_execute_named(moveit::planning_interface::MoveGroupInterface &move_group,
    rclcpp::Logger logger,
    const std::string &named_target,
    double sleep_time = 0.0)
{
move_group.setNamedTarget(named_target);
RCLCPP_INFO(logger, "Planning init trajectory...");
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

if (success)
move_group.execute(plan);
else
RCLCPP_ERROR(logger, "Planning failed!");

if (sleep_time > 0.0)
std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arm_test1");
    rclcpp::Logger logger = node->get_logger();

    // // 使用 MoveItCpp（高级接口，可获取 MoveGroupInterface）
    // moveit_cpp::MoveItCpp moveit_cpp(node);

    // 获取规划组件 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    RCLCPP_INFO(logger, "MoveItCpp and MoveGroupInterface initialized");
    // ================= Step 1: 移动到 SRDF init_pose =================
    plan_and_execute_named(move_group, logger, "init_pose", 2.0);


    // 设置目标位姿
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";  // 参考系，确保和 URDF/SRDF 匹配
    target_pose.pose.position.x = 0.3;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    target_pose.pose.orientation.w = 1.0;
  
    move_group.setPoseTarget(target_pose);

    // // 设置目标状态为预定义的 "ready"
    // std::string goal_state = "ready";
    // move_group.setNamedTarget(goal_state);

    // 规划并执行
    plan_and_execute(move_group, logger, target_pose , 3.0);

    rclcpp::shutdown();
    return 0;
}
