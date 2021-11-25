# 2021.11.03 在cyberdog环境部署BehaviorTree记录

## 编译nav2_behavior_tree失败
编译nav2_behavior_tree失败时，因为BT的库文件链接失败，导致编译失败，解决办法是手动编译behaviortree.cpp库文件，然后替换到
/opt/ros/foxy/lib/目录下面。

## nav2_rviz_plugins nav2_system_test smac_planner三个包编译失败
因为这三个包并不是我们当前需要运行环境的最小集合，所以使用skip选项选择不编译这三个包即可。
`colcon build --packages-skip nav2_rviz_plugins smac_planner nav2_system_tests`

## 在cyberdog上切换behavior tree方案的步骤
- 编译navigation2的工作空间
- 在cyberdog上进行建图并使用map_saver保存一份地图
- 使用map_server加载map，并修改配置文件映射话题名与导航公用。


下位机异常log分析

ssh root@192.168.55.233

/ mnt/UDISK/manager.log

