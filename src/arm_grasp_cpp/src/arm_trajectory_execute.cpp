#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace moveit::planning_interface;

std::vector<std::vector<double>> readCSV(const std::string& filename)
{
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open CSV file: " + filename);
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;
        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stod(value));
        }
        data.push_back(row);
    }

    file.close();
    return data;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("arm_test1");

    // 获取规划组件 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    // 读取 CSV
    std::string csv_file = "/home/tree/ros2_ws/src/dmp_pkg/config/demo_trajectory_for_discrete_dmp.csv";
    std::vector<std::vector<double>> dmp_traj = readCSV(csv_file);

    // 转换为笛卡尔轨迹点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    size_t timesteps = dmp_traj[0].size();  // CSV第一行长度
    size_t dims = dmp_traj.size();          // CSV行数，应为3

    if (dims != 3)
    {
        RCLCPP_ERROR(node->get_logger(), "CSV trajectory must have 3 rows for x,y,z.");
        return 1;
    }

    for (size_t t = 0; t < timesteps; ++t)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = dmp_traj[0][t];
        pose.position.y = dmp_traj[1][t];
        pose.position.z = dmp_traj[2][t];
        pose.orientation.w = 1.0;  // 保持默认姿态，可修改
        waypoints.push_back(pose);
    }

    // 笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;   // 每步末端移动距离
    double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(node->get_logger(), "Cartesian path computed: %f%%", fraction * 100.0);

    if (fraction < 0.99)
    {
        RCLCPP_WARN(node->get_logger(), "Trajectory not fully planned!");
    }

    // 执行轨迹
    move_group.execute(trajectory);

    rclcpp::shutdown();
    return 0;
}
