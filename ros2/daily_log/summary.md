[缩短终端用户名长度](https://zhidao.baidu.com/question/1539885970372252107.html)

ubuntu2004安装搜狗拼音   
https://blog.csdn.net/xiangbing911/article/details/109017974
https://pinyin.sogou.com/linux/help.php
https://blog.csdn.net/weixin_38924500/article/details/106156630

[ros2命令自动补全问题](https://blog.csdn.net/qq_27865227/article/details/119991497)


[C++11 std::chrono库详解](https://www.cnblogs.com/zlshmily/p/10058427.html)

[alias commands](https://blog.csdn.net/weixin_30346649/article/details/116811720)

[gazebo打不开一直卡在"Preparing your world"](https://blog.csdn.net/qq_38649880/article/details/95791253)

使用neuronbot2仿真的时候会上报serial缺失，此时需要在github上找个ros2版本的[serial](https://github.com/RoverRobotics-forks/serial-ros2)包进行使用


lifecycle节点触发机制有些拗口，使用起来有命令行可以查询设置等操作
ros2 lifecycle get/list/set/info...

ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces

colcon build --packages-select cpp_pubsub

rosdep install -i --from-path src --rosdistro foxy -y

journalctl -f | grep move_base

noname
dreame8888
123

命令行通过recovery_server调用恢复行为的方法：
```shell
ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: {x: 1.0, y: 0.0, z: 0.0}, speed: -0.3}"
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: -6.28}" 
```

启动cyberdog仿真器步骤
```shell
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models

# 启动tb3仿真器
ros2 launch  turtlebot3_gazebo turtlebot3_world.launch.py

# amcl全局定位
ros2 launch  move_base2 localization_launch.py

# amcl 或者 启动 slam
ros2 launch slam_toolbox online_async_launch.py

打开rviz，并加载rviz配置文件在move_base2中，然后设置定位信息

# amcl 或者 启动slam 并 后续保存地图等
ros2 run nav2_map_server map_saver_cli -f ~/map

# 启动导航
ros2 launch move_base2 navigation_launch_tb3.py

# lifecycle 激活
ros2 run move_base2  lifecycle_client
```

colcon build --cmake-args -DBUILD_TESTING=OFF



