# XiaoKaiBot仿真

### 1.底盤啟動

`ros2 launch xiaoKaiRobot bringup_gazebo.launch.py`

### 2.gmapping導航

* 啟動底盤 `ros2 launch xiaoKaiRobot bringup_gazebo.launch.py`
* `ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True`

### 3.保存地圖

`ros2 run nav2_map_server map_saver_cli -f maps/my_slam_box`
