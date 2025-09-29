#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  ArmTrajectoryExecute() : Node("arm_test1"){
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
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

    // 获取当前末端坐标
    this->current_pose = this-> get_current_pose_by_tf();
    auto request = std::make_shared<ArmposeToTrajectory::Request>();
    request->current_state = current_pose.position;

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
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Pose current_pose;  // 类成员

  geometry_msgs::msg::Pose get_current_pose_by_tf()
  {
    geometry_msgs::msg::Pose current_pose;
    
    try {
      // 获取从基座标系到末端坐标系的变换
      geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
          "base_link", "camera_link", 
          tf2::TimePointZero, 1s);
      
      current_pose.position.x = tf_stamped.transform.translation.x;
      current_pose.position.y = tf_stamped.transform.translation.y;
      current_pose.position.z = tf_stamped.transform.translation.z;

      current_pose.orientation = tf_stamped.transform.rotation;

      
      RCLCPP_INFO(this->get_logger(), "通过TF获取末端坐标: (%.3f, %.3f, %.3f)", 
                  current_position.x, current_position.y, current_position.z);
                  
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF变换获取失败: %s", ex.what());
    }
    return current_pose;
  }

  void trajectory_execute_callback(rclcpp::Client<ArmposeToTrajectory>::SharedFuture future)
  {
    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(), "服务返回空响应");
      return;
    }

    const auto &traj_points = response->trajectory;  // std::vector<Point>

    // 初始化 MoveGroupInterface（planning group "arm"）
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    // moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto &pt : traj_points) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation = current_pose.orientation;  // 保持初始朝向？
      waypoints.push_back(pose);
    }

    // 笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;
    double jump_threshold = 0.0;  //设成0实机有危险

    move_group->setStartStateToCurrentState();  //设置路径规划的起始状态为当前状态，很重要
    double fraction = move_group->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(),
                "笛卡尔路径规划完成度: %.2f%%",
                fraction * 100.0);

    if (fraction < 0.90) {
      RCLCPP_WARN(this->get_logger(), "轨迹规划不完整，放弃执行。");
      return;
    }

    // 执行轨迹
    auto exec_result = move_group->execute(trajectory);
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
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
