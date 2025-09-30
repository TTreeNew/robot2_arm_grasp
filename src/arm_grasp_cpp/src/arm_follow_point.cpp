#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_trajectory/robot_trajectory.h>
// #include <moveit/robot_model/robot_model.h>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/moveit_cpp/moveit_cpp.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

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

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arm_test1"); //我启动这个节点的时候使用--ros-args --params-file arm_params.yaml加载参数，用了arm_params.yaml里arm_test1这个命名空间，改动需要yaml文件一起改
    rclcpp::Logger logger = node->get_logger();

    // // 使用 MoveItCpp（高级接口，可获取 MoveGroupInterface）
    // moveit_cpp::MoveItCpp moveit_cpp(node);

    // 获取规划组件 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    RCLCPP_INFO(logger, "MoveItCpp and MoveGroupInterface initialized");

    // 设置目标位姿
    geometry_msgs::msg::PoseStamped target_pose0,target_pose1,target_pose2,target_pose3;
    target_pose0.header.frame_id = "base_link";   //直立
    target_pose0.pose.position.x = -0.007;
    target_pose0.pose.position.y = -0.091;
    target_pose0.pose.position.z = 0.733;
    target_pose0.pose.orientation = euler_to_quaternion(-0.014,-1.543,-1.564);

    target_pose1.header.frame_id = "base_link";  // 第一个目标点
    target_pose1.pose.position.x = -0.046;
    target_pose1.pose.position.y = -0.463;
    target_pose1.pose.position.z = 0.033;
    target_pose1.pose.orientation = euler_to_quaternion(0.321, 1.490, -1.383);
    // target_pose1.pose.orientation.x = 0;
    // target_pose1.pose.orientation.y = 0;
    // target_pose1.pose.orientation.z = 0;
    // target_pose1.pose.orientation.w = 1;

    target_pose2.header.frame_id = "base_link"; //过度1
    target_pose2.pose.position.x = 0.099;
    target_pose2.pose.position.y = -0.513;
    target_pose2.pose.position.z = 0.314;
    target_pose2.pose.orientation = euler_to_quaternion(0.052, 0.261, -1.368);

    target_pose3.header.frame_id = "base_link"; //过度2
    target_pose3.pose.position.x = -0.082;
    target_pose3.pose.position.y = -0.475;
    target_pose3.pose.position.z = 0.449;
    target_pose3.pose.orientation = euler_to_quaternion(-0.004, -0.677, -1.570);    

    geometry_msgs::msg::PoseStamped target_pose4,target_pose5; 
    target_pose4.header.frame_id = "base_link";    //过度3
    target_pose4.pose.position.x = 0.478;
    target_pose4.pose.position.y = -0.257;
    target_pose4.pose.position.z = 0.256;
    target_pose4.pose.orientation = euler_to_quaternion(0.065, 0.717, -0.456); 

    target_pose5.header.frame_id = "base_link";    // 第二个目标点
    target_pose5.pose.position.x = 0.498;
    target_pose5.pose.position.y = -0.031;
    target_pose5.pose.position.z = 0.068;
    target_pose5.pose.orientation = euler_to_quaternion(0.317, 1.461, 0.226); 

 

    plan_and_execute(move_group, logger, target_pose0 , 1.0);    

    plan_and_execute(move_group, logger, target_pose1 , 1.0);     // 规划并执行

    plan_and_execute(move_group, logger, target_pose2 , 1.0);    

    plan_and_execute(move_group, logger, target_pose3 , 1.0); 

    plan_and_execute(move_group, logger, target_pose4 , 1.0);   

    plan_and_execute(move_group, logger, target_pose5 , 1.0);   

    rclcpp::shutdown();
    return 0;
}
