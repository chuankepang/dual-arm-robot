### 1. 使用左臂机器人

依次运行以下命令，接着即可使用`rviz`的可视化界面控制真实的左臂UR10机械臂系统。

```shell
# 与左臂机械臂建立连接
roslaunch ur_modern_driver left_dual_ur_e_bringup.launch
# 启动左臂机械臂gazebo
roslaunch dual_ur_gazebo left_dual_ur_e.launch
# 启动move_group
roslaunch dual_ur_moveit_config dual_ur_moveit_planning_execution.launch
# 启动rviz可视化运动规划
roslaunch dual_ur_moveit_config moveit_rviz.launch 
# 启动轨迹规划与手爪脚本
rosrun ur_moveit left_dual_assembly_1.0.py
```


### 2. 使用双臂机器人

依次运行以下命令，接着即可使用`rviz`的可视化界面控制真实的双臂UR10机械臂系统。

```shell
# 与双机械臂建立连接
roslaunch ur_modern_driver dual_ur_e_bringup.launch
# 启动双臂机械臂gazebo
roslaunch dual_ur_gazebo dual_ur_e.launch
# 启动move_group
roslaunch dual_ur_moveit_config dual_ur_moveit_planning_execution.launch limited:=true
# 启动rviz可视化运动规划
roslaunch dual_ur_moveit_config moveit_rviz.launch 
# 启动轨迹规划与手爪脚本
rosrun ur_moveit dual_assembly_1.0.py
```
