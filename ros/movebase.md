# move_base

## 概念

> move_base包为一个ation提供了实现的途径（详情参见actionlib包)。假如在地图内给定一个目标，这个操作将尝试控制一个移动基座到达这个目标。

>move_base节点将全局路径和局部路径规划程序链接在一起，以完成其全局导航任务。

>它支持连接符合nav_core包中指定的nav_core::BaseGlobalPlanner接口的任一全局路径规划器和符合nav_core::BaseLocalPlanner接口的任一局部路径规划器。

>move_base节点还维护两个costmaps，一个用于全局路径规划器，一个用于局部路径规划器（参见costmap_2d软件包），用于完成导航任务。

>move_base为使用导航功能包集将机器人移动到所需的位置提供了一个总入口，将外部传感器数据，tf关系，定位，地图，costmap，planner，recovery连贯起来

## 逻辑框架图

![导航栈逻辑框架图](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf_small.png)

>说明：

>> move_base节点的高级视图及其与其他组件的交互如上所示。蓝色部分基于机器人平台而变化，所有平台都能提供可选择的灰色部分和不可少的白色部分。


## move_base节点进行流程

![move_base节点进行流程](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=recovery_behaviors.png)

### 机器人详细行为流程

>在机器人尝试到达在使用者允许的误差范围内目标位置。

>在没有动态障碍的情况下，move_base节点最终运行结果要么是接近了规划的目标，要么是给用户返回失败信号。

>当机器人认为它是卡住了，move_base节点可以选择执行修复操作。默认情况下，move_base节点采用如下所述行动来清理出的周围空间：

>>用户指定区域以外的障碍物将从机器人的地图中清除。如果失败，机器人会更激进的将其可以就地旋转的矩形框外面的障碍物全部移除。

>>紧接着下一个就地旋转操作会被继续执行。如果所有这些操作都失效，机器人会认为目标设定不恰当并通知用户放弃操作.

>>这些修复操作可以通过recovery_behaviors参数来配置，也可以recovery_behavior_enabled参数disable。

## move base action

定义在move_base_msg包里面的action协议。用于指定导航目标点位置

## MoveBase类资源结构

 ![MoveBase资源](/home/ld/native/navigation/move_base/image/resource.png)

