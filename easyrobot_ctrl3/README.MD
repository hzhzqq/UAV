# 配置mavlink mavros
src/mavros/mavros/launch/px4_pluginlists.yaml的plugin_blacklist:中注释掉：

```
 global_position
 setpoint_attitude
 setpoint_position
 setpoint_accel
 setpoint_velocity
```


# 编译
catkin_make -DCATKIN_BLACKLIST_PACKAGES="easyrobot_bringup;easyrobot_demo"
这两个包编译会出错
# 启动仿真环境 
roslaunch easyrobot_simulation circle_cross.launch

# 启动QGC 
启动仿真后在QGC中设置以下参数

MPC_XY_CRUISE 设置为1
MPC_XY_VEL_MAX 设置为1，飞的慢一些
do_offboard_takeoff()函数中，调整起飞航点 pose.pose.position 来对准圆环。

# 穿圆demo
## 启动仿真环境 
roslaunch easyrobot_simulation circle_cross.launch

## 启动视觉识别
roslaunch spirecv_ros ellipse_detection_with_cv_bridge.launch

## 启动穿圆控制
rosrun easyrobot_ctrl3 easyrobot_ctrl3_node
无人机将自动起飞、穿圆，自动降落。

请将无人机正对圆，如果偏置，无人机可能会撞在圆环上。

# 精准降落demo
## 启动仿真环境 
roslaunch easyrobot_simulation aruco_land.launch
## 启动视觉识别
roslaunch spirecv_ros easyrobot_aruco_detect.launch

## 启动穿圆控制
rosrun easyrobot_ctrl3 easyrobot_ctrl3_node 

