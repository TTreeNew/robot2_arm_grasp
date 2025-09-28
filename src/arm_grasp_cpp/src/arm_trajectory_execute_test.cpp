#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "my_robot_interfaces/srv/armpose_to_trajectory.hpp"  

#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>


// 使用时间单位的字面量，可以在代码中使用 s 和 ms 表示时间
using namespace std::chrono_literals;
using ArmposeToTrajectory = my_robot_interfaces::srv::ArmposeToTrajectory;

class ArmTrajectoryExecute : public rclcpp::Node
{
public:
  ArmTrajectoryExecute() : Node("arm_test1")
  {
    trajectory_execute_client = this->create_client<ArmposeToTrajectory>("armpose_to_trajectory");
    // 1.等待服务端上线
    while (!trajectory_execute_client->wait_for_service(1s))
    {
      // 等待时检测rclcpp的状态
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 初始化 MoveGroupInterface（planning group "arm"）
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    // moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");
    // 获取当前末端坐标
    auto pose = move_group->getCurrentPose();
    auto request = std::make_shared<ArmposeToTrajectory::Request>();
    request->current_state.x = pose.pose.position.x;
    request->current_state.y = pose.pose.position.y;
    request->current_state.z = pose.pose.position.z;

    RCLCPP_INFO(this->get_logger(),
                "当前末端坐标: (%.3f, %.3f, %.3f)",
                request->current_state.x,
                request->current_state.y,
                request->current_state.z);

    // 发送异步请求，并绑定回调
    trajectory_execute_client->async_send_request(
        request,
        std::bind(&ArmTrajectoryExecute::trajectory_execute_callback,
                  this,
                  std::placeholders::_1));
  }

private:
  rclcpp::Client<ArmposeToTrajectory>::SharedPtr trajectory_execute_client;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

  void trajectory_execute_callback(rclcpp::Client<ArmposeToTrajectory>::SharedFuture future)
  {
    auto response = future.get();
    const auto &traj_points = response->trajectory;  // std::vector<Point>

    // 转换为笛卡尔轨迹点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto &pt : traj_points) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1.0;  // 简单保持默认姿态
      waypoints.push_back(pose);
    }

    // 笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;
    double jump_threshold = 0.0;

    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(),
                "笛卡尔路径规划完成度: %.2f%%",
                fraction * 100.0);

    if (fraction < 0.90) {
      RCLCPP_WARN(this->get_logger(), "轨迹规划不完整，放弃执行。");
      return;
    }

    // 执行轨迹
    auto exec_result = move_group_->execute(trajectory);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "轨迹执行成功。");
    } else {
      RCLCPP_ERROR(this->get_logger(), "轨迹执行失败。");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmTrajectoryExecute>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
