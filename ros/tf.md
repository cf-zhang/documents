# tf

>tf是一个允许用户随时间跟踪多个坐标框架的包。 tf维持时间缓冲的树结构中的坐标帧之间的关系，并允许用户在任何期望的时间点在任何两个坐标帧之间变换点，向量等

>tf2是tf的迭代升级版本

## tf是用来做什么的，我们怎么使用tf

>机器人系统通常具有许多随时间变化的3D坐标框架，例如世界框架，基础框架，抓手框架，头架等。随着时间的推移跟踪所有这些框架，并允许您提出如下问题：

>>1. 5秒前，头架相对于世界框架在哪里？

>>2. 我的抓手中的物体相对于我的底座的姿势是什么？

>>3. 地图框架中基础框架的当前姿势是什么？

>>4. 支持分布式部署


>

>

>



# tf简介

>rosrun tf view_frames  && evince frames.pdf  会生成一个当前系统中的tf关系pdf文档并进行查看

>rosrun rqt_tf_tree rqt_tf_tree

>rosrun tf tf_echo [reference_frame] [target_frame]



# tf设计理念

## 设计目标

>分布式系统

>仅在使用时在坐标框架之间转换数据	

>支持对除当前时间以外的时间戳加时间戳的数据进行查询	

>只需知道要处理数据的坐标系的名称

>系统不需要事先知道配置，并且可以动态地处理重新配置

>核心是ROS不可知的

>线程安全的接口

>支持多个机器人

>>对每个机器人使用类似于命名空间的tf_prefix。

## 已知的限制

>tf_prefix令人困惑且违反直觉

>图表的方向可能令人困惑

>消息不能很好地处理低带宽网络

>tf不能保存长时间的历史记录

>> 存储的默认时间刻度是10秒，这对于实时操作来说很好，但是存储比这更长的任何东西需要将数据存储在已知的固定帧中，以便稍后可以变换。
	
# tf教程





