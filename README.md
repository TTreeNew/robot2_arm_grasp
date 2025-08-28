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
### 对原ros1的urdf文件修改
1. urdf文件里删去了虚拟关节world，虚拟关节world在moveit2调试助手里声明(即在launch文件里发布world到机械臂的tf变换)，关节名称virtual_joint，父坐标系world。
2. 原项目urdf文件中虽然有ros_control插件，但是似乎并没有使用ros_control控制，使用的是gazebo的`gazebo_msgs/SetModelState.h`，通过向`/gazebo/set_model_state`发布消息控制机械臂。这里改为使用ros2_control。
3.  `<gazebo>`里的激光雷达插件换成了libgazebo_ros_ray_sensor.so
4.   urdf文件里的gazebo标签里的插件标签用的驼峰命名，不知道有没有问题，我改成下划线命名了，比如：`<frameName>`变成`<frame_name>`。并且删除了`<gazebo>`里link_7标签。
5.   原本使用`<transmition>`配置硬件接口，我换成了组件化硬件接口的写法：即在`<ros2_control>`中定义接口。删去`<transmition>`标签

### moveit配置助手操作简单说明
1. camera在最末端所以把他设置成末端执行器，对应的group name和end effector name都取名hand。
2. 调试助手中，规划组取名arm，全部放入arm这一个规划组，初始姿态取名init_pose。passive joint跳过。
3. ros2_controller使用位置控制，控制器选择`joint_trajectory_controller/JointTrajectoryController`，
4. controller name 自动生成，自动生成的名字就是 arm_controller,这个名字就是ros2_controller配置文件里给插件取的名字，最好和moveit controllers里取的名字一样。控制器类型默认的`joint_trajectory_controller/JointTrajectoryController`。
5. moveit控制器也叫ros2_arm_controller，名字不一致需要自己修改action_ns（不知道moveit配置助手好像不会修改action_ns）（这里的 FollowJointTrajectory 是告诉moveit通过`/arm_joint_trajectory_controller/follow_joint_trajectory`这个action服务向ros2_controller发送消息控制机械臂。FollowJointTrajectory是这个接口使用的action消息类型的名字）
### rviz2验证之前对配置助手生成文件的修改
1. 找到生成的moveit_controllers.yaml文件，arm_controllers如果没有写action_ns，手动补上
```yaml
action_ns: follow_joint_trajectory
default: true
```
不然执行轨迹的时候找不到arm控制器无法完成execute。

### gazebo仿真之前的修改
1.   把`<mesh filename="package://arm_description/meshes/base_link.STL" />`这种写mesh文件路径方式换成`<mesh filename="file://$(find arm_description)/meshes/base_link.STL" />`，这样之后rviz2和gazebo都能正常显示，否则gazebo无法解析mesh文件路径。但是这样之后moveit2调试助手又无法显示模型了，如果要使用moveit2调试助手又要改回原来的package://写法。
2.  把`src/arm_moveit_config/config/ar3.ros2_control.xacro`里的`mock_components/GenericSystem`插件改成`gazebo_ros2_control/GazeboSystem`
3. 把
`src/arm_moveit_config/config/moveit_controllers.yaml`里的`moveit_simple_controller_manager/MoveItSimpleControllerManager`改成`moveit_ros2_controller_manager/MoveItRos2ControllerManager`
4. 找到moveit生成的config文件夹下的ros2_control.xacro，把里面的
`<xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>`里的`load_yaml(`
替换成
`xacro.load_yaml(`。
