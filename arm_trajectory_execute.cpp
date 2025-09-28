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

using namespace moveit::planning_interface;


class ArmTrajectoryExecute : public rclcpp::Node
{
public:
    explicit ArmTrajectoryExecute(const std::string & node_name) : Node(node_name)  //"arm_trajectory_execute"
    {

    }
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
private:

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmTrajectoryExecute>("arm_test1"); //我启动这个节点的时候使用--ros-args --params-file arm_params.yaml加载参数，用了arm_params.yaml里arm_test1这个命名空间，改动需要yaml文件一起改
    // auto moveit_node = rclcpp::Node::make_shared("arm_test1"); //我启动这个节点的时候使用--ros-args --params-file arm_params.yaml加载参数，用了arm_params.yaml里arm_test1这个命名空间，改动需要yaml文件一起改
    rclcpp::Logger logger = node->get_logger();
    // 获取规划组件 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    RCLCPP_INFO(logger, "MoveItCpp and MoveGroupInterface initialized");
    // 读取 CSV
    std::string csv_file = "/home/tree/robot2_arm_grasp/src/dmp_pkg/config/demo_trajectory_for_discrete_dmp.csv";
    std::vector<std::vector<double>> dmp_traj = node->readCSV(csv_file);
    
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
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    double eef_step = 0.01;   // 每步末端移动距离
    double jump_threshold = 0;
    
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);
    RCLCPP_INFO(node->get_logger(), "Cartesian path computed: %f%%", fraction * 100.0);
    
    if (fraction < 0.90)
    {
        RCLCPP_WARN(node->get_logger(), "Trajectory not fully planned!");
        return 1;
    }
    
    // 执行轨迹
    move_group.execute(trajectory_msg);

    rclcpp::spin(node);    
    rclcpp::shutdown();
}

    
    // // 等待 current state 就绪
    // rclcpp::Time start_time = node->now();
    // const double wait_timeout = 5.0;
    // while (rclcpp::ok()) {
    //     auto current_state = move_group.getCurrentState();
    //     if (current_state && current_state->getVariableCount() > 0) {
    //         break;
    //     }
    //     if ((node->now() - start_time).seconds() > wait_timeout) {
    //         RCLCPP_ERROR(node->get_logger(), "Timeout waiting for valid current robot state. Check /joint_states and /clock.");
    //         return 1;
    //     }
    //     RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Waiting for current_state...");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    
    // // 以当前状态为 start
    // move_group.setStartStateToCurrentState();
    
    // // 将当前末端位姿插入 waypoints 开头（提高逐点 IK 连续性）
    // geometry_msgs::msg::PoseStamped cur_ps = move_group.getCurrentPose();
    // waypoints.insert(waypoints.begin(), cur_ps.pose);



    
    // // 转成 RobotTrajectory
    // auto robot_model = move_group.getRobotModel();
    // robot_trajectory::RobotTrajectory trajectory(robot_model, "arm");
    // trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);
    
    // // 时间参数化
    // trajectory_processing::IterativeParabolicTimeParameterization time_param;
    // double velocity_scaling_factor = 0.1;     // 你 yaml 里也设置过
    // double acceleration_scaling_factor = 0.1; // 你 yaml 里也设置过
    // bool success = time_param.computeTimeStamps(trajectory, velocity_scaling_factor, acceleration_scaling_factor);
    // if (!success)
    // {
    //     RCLCPP_WARN(node->get_logger(), "Time parameterization failed");
    // }
    
    // // 再转回 moveit_msgs::msg::RobotTrajectory 执行
    // trajectory.getRobotTrajectoryMsg(trajectory_msg);
    // // move_group.execute(trajectory_msg);
    
    // return 0;
        
        
        