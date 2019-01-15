# Remapping Arguments



在命令行启动节点时，可以重新映射节点中的任何ROS名称。这是ROS的一个强大功能，允许您从命令行在多个配置下启动同一个节点。可以重新映射所有资源名称。ROS的这个特性允许您将复杂的名称分配推迟到系统的实际运行时加载。 



可以将重新映射参数传递给任何节点，syntax_name：=new_name。例如，要将Talker节点配置为发布到/wg/chatter而不是chatter，请执行以下操作：
```$xslt
rosrun rospy_tutorials talker chatter:=/wg/chatter
``` 

只能匹配名字，即最后的一段名字：foo可以匹配  /foo、 foo、 /bar/foo不能匹配：/foo/***

## pushing down

launch文件里面使用ROS_NAMESPACE变量可以将所有的被包围的名字进行向下缩进。

避免使用全局名称，推荐使用相对和私有名字

## 参数重映射

_param:=1.0

将节点内的参数～param设置为1.0

## 特殊的关键字

>__name  节点的名字

>__ip 、__hostname  替代系统变量中默认的 ROS_IP ROS_HOSTNAME

>__master 替代系统变量中默认的ROS_MASTER_URI

>__ns 替代系统变量中默认的ROS_NAMESPACE

## 私有名字

```$xslt


<launch>
    <node name="param" pkg="mypkg" type="param.py" output="screen">
        <remap from="/bar" to="/foo" />
        <remap from="~baz" to="foo" />
    </node>
</launch>

```