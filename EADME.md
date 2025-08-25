[原项目参考地址](https://www.bilibili.com/video/BV1kypFepEfb/?vd_source=c46697e1d06be94cd5e10883ded86efe)

## 简介
原项目是基于ros1的机械臂抓取项目。
用ros2实现与原项目相差太大。我用ros2来实现，现在只完成了仿真文件的编写。

## 运行

进入ros2_arm_ws目录。在终端中打开。
运行source install/setup.bash 和 ros2 launch arm_description gazebo_sim.launch.py ，启动gazebo仿真环境
打开新的终端运行rviz2，添加 RobotModel插件，调整Fixed Frame 和 topic 
(待补充)

## 修改
此为原项目的重写而非移植，按照原代码的思路做了一些调整
1. urdf文件里删去了虚拟关节world，虚拟关节在moveit2调试助手里声明，关节名称virtual_joint，父坐标系world。
2. 原项目urdf文件中虽然有ros_control插件，但是似乎并没有使用ros_control控制，使用的是gazebo的`gazebo_msgs/SetModelState.h`，通过向`/gazebo/set_model_state`发布消息控制机械臂。这里改为使用ros2_control。
3. camera在最末端所以把他设置成末端执行器，用add Links添加，对应的group name和end effector name都取名hand。
4. 调试助手中，规划组取名arm，初始姿态取名init_pose。passive joint只添加的joint6。
5. ros2_control使用位置控制`joint_trajectory_controller/JointTrajectoryController`，读取关节位置和速度
6. controller name 取名 ros2_arm_controller,控制器类型默认的`joint_trajectory_controller/JointTrajectoryController`。
7. moveit控制器也叫ros2_arm_controller。
8. 找到moveit生成的config文件夹下的ros2_control.xacro，把里面的
`<xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>`
替换成了
`<xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>`，否则gazebo无法加载
9. `<gazebo>`里的激光雷达插件换成了libgazebo_ros_ray_sensor.so
10. urdf文件里的gazebo标签里的插件标签用的驼峰命名，不知道有没有问题，我改成下划线命名了，比如：`<frameName>`变成`<frame_name>`。并且删除了`<gazebo>`里link_7标签
11. 原本使用`<transmition>`配置硬件接口，我换成了组件化硬件接口的写法：即在`<ros2_control>`中定义接口。删去`<transmition>`标签
12. 把`<mesh filename="package://arm_description/meshes/base_link.STL" />`这种写mesh文件路径方式换成`<mesh filename="file://$(find arm_description)/meshes/base_link.STL" />`，这样之后rviz2和gazebo都能正常显示，否则gazebo无法解析mesh文件路径。但是这样之后moveit2调试助手又无法显示模型了，如果要使用moveit2调试助手又要改回原来的package://写法。
13、把`src/arm_moveit_config/config/ar3.ros2_control.xacro`里的`mock_components/GenericSystem`插件改成`gazebo_ros2_control/GazeboSystem`，并且把
`src/arm_moveit_config/config/moveit_controllers.yaml`里的`moveit_simple_controller_manager/MoveItSimpleControllerManager`改成`moveit_ros2_controller_manager/MoveItRos2ControllerManager`
此时已经不需要`initial_positions.yaml`了，所以使用`initial_positions.yaml`的部分同步修改
（这部分内容待简化）
