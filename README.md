[原项目参考地址](https://www.bilibili.com/video/BV1kypFepEfb/?vd_source=c46697e1d06be94cd5e10883ded86efe)

## 简介
原项目是基于ros1的机械臂抓取项目。
用ros2实现与原项目相差太大。我用ros2来实现，现在完成了仿真文件的编写。并且用cpp接口控制gazebo的机械臂。
借用了他的机械臂模型和mesh文件。

## 文件说明
- arm_moveit_config文件是(moveit2配置助手)moveit2 setup assistant 生成的文件
- arm_description包含机器人模型和启动gazebo，rviz2的launch文件
- full_demo.launch.py 可以启动rviz2，具体启动内容见launch文件里的return LaunchDescription([………………])。是在纯rviz2里验证是否配置成功的，现在无法运行原因见"gazebo仿真之前的修改"部分。
- gazebo_sim.launch.py 可以启动gazebo并加载控制器和机械臂。不能加载move_group。
- sim_with_moveit.launch.py可以启动gazebo和rviz2 ，是对gazebo_sim.launch.py和full_demo.launch.py的综合。
- launch文件中的full_demo.launch.py为对配置助手生成的demo.launch.py的重写，demo.launch.py非常不直观，所以重写，两者是等价的。[full_demo.launch.py参考地址](https://www.bilibili.com/video/BV1CshJzdEpU?spm_id_from=333.788.player.switch&vd_source=bffdb80b975508dc5f2dd69ec6999b3b)。
  
## 运行
把models文件夹复制到.gazebo里面和里面的models文件夹合并  
启动launch前可以修改launch文件里rviz2里加载.rviz配置文件的地址。这样就能选择打开rviz2时加载的是motionplanning插件还是robotmodel插件，用robotmodel插件才能看到rviz的机械臂随着gazebo的机械臂一起动。

鼠标点击进入ros2_arm_ws目录。在终端中打开。运行下面命令，启动gazebo仿真环境。
```bash
source install/setup.bash 
ros2 launch arm_description sim_with_moveit.launch.py
```

接着在ros2_arm_ws目录下打开新的终端。运行下面命令，对机械臂进行简单的控制
```bash
source install/setup.bash 
ros2 run arm_grasp_cpp arm_test1 --ros-args --params-file arm_params.yaml
```

记录末端笛卡尔坐标系下的轨迹，按ctrl c 停止节点并导出demo_trajectory_for_discrete_dmp.csv文件到dmp_pkg/config：
```bash
source install/setup.bash 
ros2 run dmp_pkg arm_trajectory_record 
```
按点控制则启动：
```bash
source install/setup.bash 
ros2 run arm_grasp_cpp arm_follow_point --ros-args --params-file arm_params.yaml
```

服务的客户端，发送机械臂当前位置给服务端获得生成的dmp轨迹并执行：
```bash
source install/setup.bash 
ros2 run arm_grasp_cpp arm_trajectory_execute --ros-args --params-file arm_params.yaml
```
加载教学轨迹并保存权重：
```bash
source install/setup.bash 
ros2 run dmp_pkg save_weight
```
服务的服务端，接收机械臂末端当前位置作为起始位置。然后加载保存的权重生成对应的dmp轨迹：
```bash
source install/setup.bash 
ros2 run dmp_pkg trajectory_gen_pub
```

(待补充)

## 修改
此为原项目的重写而非移植，按照原代码的思路做了一些调整
### 对原ros1的urdf文件修改
1. urdf文件里删去了虚拟关节world，虚拟关节world在moveit2调试助手里声明(即在launch文件里发布world到机械臂的tf变换)，关节名称virtual_joint，父坐标系world。（如果是固定基座好像不能删？）
2. 原项目urdf文件中虽然有ros_control插件，但是似乎并没有使用ros_control控制，使用的是gazebo的`gazebo_msgs/SetModelState.h`，通过向`/gazebo/set_model_state`发布消息控制机械臂。这里改为使用ros2_control。
3.  `<gazebo>`里的激光雷达插件换成了libgazebo_ros_ray_sensor.so
4.   urdf文件里的gazebo标签里的插件标签用的驼峰命名，不知道有没有问题，我改成下划线命名了，比如：`<frameName>`变成`<frame_name>`。并且删除了`<gazebo>`里link_7标签。
5.   原本使用`<transmition>`配置硬件接口，我换成了组件化硬件接口的写法：即在`<ros2_control>`中定义接口。删去`<transmition>`标签

### moveit配置助手操作简单说明
1. camera在最末端所以把他设置成末端执行器，对应的group name和end effector name都取名hand。
2. 调试助手中，规划组取名arm，全部放入arm这一个规划组，初始姿态取名init_pose。passive joint跳过。
3. ros2_controller使用位置控制，控制器选择`joint_trajectory_controller/JointTrajectoryController`，
4. controller name 自动生成，自动生成的名字就是 arm_controller,这个名字就是ros2_controller配置文件里给插件取的名字，最好和moveit controllers里取的名字一样。控制器类型默认的`joint_trajectory_controller/JointTrajectoryController`。
5. moveit控制器也叫ros2_arm_controller，名字不一致需要自己修改action_ns（moveit配置助手好像不会修改action_ns）（这里的 FollowJointTrajectory 是告诉moveit通过`/arm_joint_trajectory_controller/follow_joint_trajectory`这个action服务向ros2_controller发送消息控制机械臂。FollowJointTrajectory是这个接口使用的action消息类型的名字）
### rviz2验证之前对配置助手生成文件的修改
1. 找到生成的moveit_controllers.yaml文件，arm_controllers如果没有写action_ns，手动补上
```yaml
action_ns: follow_joint_trajectory
default: true
```
不然执行轨迹的时候找不到arm控制器无法完成execute。

### gazebo仿真之前的修改
1.   由于我使用的是gazebo11，所以把`<mesh filename="package://arm_description/meshes/base_link.STL" />`这种写mesh文件路径方式换成`<mesh filename="file://$(find arm_description)/meshes/base_link.STL" />`，这样之后rviz2和gazebo11都能正常显示，否则gazebo11无法解析mesh文件路径。但是这样之后moveit2调试助手又无法显示模型了，如果要使用moveit2调试助手又要改回原来的package://写法。查找（ctrl F） `"package://arm_description` 换成 `"file://$(find arm_description)`
2.  把`src/arm_moveit_config/config/ar3.ros2_control.xacro`里的`mock_components/GenericSystem`插件改成`gazebo_ros2_control/GazeboSystem`
3. 把
`src/arm_moveit_config/config/moveit_controllers.yaml`里的`moveit_simple_controller_manager/MoveItSimpleControllerManager`改成`moveit_ros_control_interface/Ros2ControlManager`
4. 找到moveit生成的config文件夹下的ros2_control.xacro，把里面的
`<xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>`里的`load_yaml(`
替换成
`xacro.load_yaml(`。
(使用gazebo+rviz2时注意{'use_sim_tim': True})
5. 工作空间下的`arm_params.yaml`是为了单独用run来启动arm_test1.cpp生成的。这样就不用在我的`sim_with_moveit.launch.py`里添加启动arm_test1.cpp节点的代码了。生成方法是启动`sim_with_moveit.launch.py`之后运行
```bash
ros2 param dump /move_group > arm_params.yaml
```
然后①把生成的yaml文件最外层的命名空间换成 arm_test1 ，即把 ros__parameters 上面的那个 move_group 换成 arm_test1 ②把除
```yaml
arm: 
  kinematics_solver: null
```
外所有的值为null的行全部删除③把`kinematics_solver: null`换成`kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin`

6. launch文件里启动gazebo的时候需要在参数那里显式设置('use_sim_time', 'true'),gazebo才会发布/clock 。如果gazebo不发布/clock ，其他use_sim_time的节点不能获得 sim_time
    
