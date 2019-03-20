
使用启动文件或绑定和服务向聚合器添加一组新的分析程序。
## 概述
可以修改诊断聚合器在运行时分析的内容。这允许您仅在需要时临时添加诊断分析程序，以减少混乱，或者添加与启动文件中的特定节点或节点集合相对应的分析程序，以便在运行启动文件时添加它们。这还有一个好处，允许您根据分析系统的哪些部分来分离分析程序。

本教程将向您展示如何从启动文件或节点内部添加分析程序。我们将创建一个小测试包来演示该功能。
## 创建一个测试包
```
catkin_create_pkg test_add_analyzers diagnostic_msgs rospy roscpp bondpy
```
为启动文件创建目录，为诊断发布创建脚本，为分析器设置创建参数。
```
roscd test_add_analyzers
mkdir launch scripts param
```
为诊断聚合器添加一个启动文件。

launch/base.launch:
```
<launch>
  <node pkg="test_add_analyzers" type="base_node.py" name="base_node"/>
  <node pkg="diagnostic_aggregator" type="aggregator_node" name="diagnostic_aggregator">
    <rosparam command="load" file="$(find test_add_analyzers)/param/base_analyzers.yaml"/>
  </node>
</launch>
```
添加一个节点来发布一些诊断消息。不要忘记让它可执行。

scripts/ base_node.py
```
#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

if __name__ == '__main__':
    rospy.init_node('base_node')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    arr = DiagnosticArray()
    arr.status = [
        DiagnosticStatus(name='battery battery_message', message='the battery message'),
        DiagnosticStatus(name='joystick joystick_message', message='the joystick message')
    ]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        arr.header.stamp = rospy.Time.now()
        pub.publish(arr)
        rate.sleep()
```
添加一个.yaml，其中包含一些简单的分析程序。
param/base_analyzers.yaml:
```
analyzers:
  base:
    type: diagnostic_aggregator/AnalyzerGroup
    path: Base
    analyzers:
      battery:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Battery
        find_and_remove_prefix: 'battery'
      joy:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Joystick
        find_and_remove_prefix: 'joystick'
```

## 从启动文件加载
从启动文件加载与启动诊断聚合器的方式类似。定义分析程序的参数被直接加载到节点名称空间中，然后聚合器读取节点名称空间以初始化新的分析程序。add_analyzers节点通过一个bond对象链接到聚合器。当节点关闭时，键断开，与该节点关联的分析程序将从聚合器中删除。当节点关闭时，节点加载的参数不会被删除。

添加一个启动文件来添加分析程序。

launch/simple.launch
```
<launch>
  <node pkg="test_add_analyzers" type="simple_node.py" name="simple_node"/>
  <node pkg="diagnostic_aggregator" type="add_analyzers" name="add_simple_analyzers">
    <rosparam command="load" file="$(find test_add_analyzers)/param/simple_analyzers.yaml" />
  </node>
</launch>
```
添加一个节点来将一些虚拟数据发布到诊断主题。记住要让它可执行。
scripts/simple_node.py
```
#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

if __name__ == '__main__':
    rospy.init_node('simple_node')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    arr = DiagnosticArray()
    arr.status = [
        DiagnosticStatus(name='rgb_stream rgb_message', message='the rgb data'),
        DiagnosticStatus(name='rgbd_stream rgbd_message', message='the rgbd data')
    ]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        arr.header.stamp = rospy.Time.now()
        pub.publish(arr)
        rate.sleep()
```
为一些不同的分析器规范添加.yaml。

param/simple_analyzers.yaml

```
analyzers:
    vision:
      type: diagnostic_aggregator/AnalyzerGroup
      path: Vision
      analyzers:
        rgb:
          type: diagnostic_aggregator/GenericAnalyzer
          path: CameraRGB
          find_and_remove_prefix: 'rgb_stream'
        rgbd:
          type: diagnostic_aggregator/GenericAnalyzer
          path: CameraRGBD
          find_and_remove_prefix: 'rgbd_stream'
```
首先，使用基本分析程序启动聚合器
```
roslaunch test_add_analyzers base.launch
```
然后，运行rqt_robot_monitor来查看聚合器输出。
```
rosrun rqt_robot_monitor rqt_robot_monitor
```

最后，将简单的分析程序添加到聚合器输出中。
```
roslaunch test_add_analyzers simple.launch
```
 您应该会看到诊断输出出现，首先出现在另一个组中，然后在聚合器完成更新分析程序时出现在正确的组中。
 
## 从一个节点中加载
还可以从节点内部向聚合器添加分析程序。

为节点创建启动程序。

launch/manual_node.launch
```
<launch>
  <node pkg="test_add_analyzers" type="manual_diag.py" name="manual_diag"/>
  <node pkg="test_add_analyzers" type="manual_node.py" name="manual_node">
    <param name="analyzer_yaml" value="$(find test_add_analyzers)/param/manual_analyzers.yaml" />
  </node>
</launch>
```
添加一些分析器参数。

param/manual_analyzers.yaml
```
analyzers:
  buttons:
    type: diagnostic_aggregator/AnalyzerGroup
    path: Buttons
    analyzers:
      go_button:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Go
        find_and_remove_prefix: 'button_go'
      stop_button:
        type: diagnostic_aggregator/GenericAnalyzer
        path: Stop
        find_and_remove_prefix: 'button_stop'
```

添加一个节点来发布诊断信息

scripts/manual_diag.py
```
#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

if __name__ == '__main__':
    rospy.init_node('manual_diag')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    arr = DiagnosticArray()
    arr.status = [
        DiagnosticStatus(name='button_stop go_message', message='the go button message'),
        DiagnosticStatus(name='button_go stop_message', message='the stop button message')
    ]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        arr.header.stamp = rospy.Time.now()
        pub.publish(arr)
        rate.sleep()
```


添加一个节点来执行添加操作。


scripts/manual_node.py
```
#!/usr/bin/env python

import rospy
import rosparam
from bondpy import bondpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from diagnostic_msgs.srv import AddDiagnostics

class ManualNode(object):

    def __init__(self):
        rospy.init_node('manual')
        rospy.on_shutdown(self.shutdown)
        # Load yaml parameters from a file, and load them into the parameter server
        # under this node's namespace.
        paramlist = rosparam.load_file(rospy.get_param('~analyzer_yaml'))
        for params, ns in paramlist:
            rosparam.upload_params(rospy.get_name() + '/' + ns, params)

        # Can use any namespace into which the parameters that we want to use to
        # specify analyzers have been loaded
        self.namespace = rospy.get_name()


    def start(self):

        # Create a bond to try and connect to the diagnostics aggregator. The second
        # parameter to the bond is the name of the bond. This name should be the
        # same as the load_namespace below
        self.bond = bondpy.Bond("/diagnostics_agg/bond", rospy.resolve_name(self.namespace))

        # Start the bond, initialising the connection and creating the analyzers
        # that were defined by the parameters loaded earlier
        self.bond.start()

        # To start the other side of the bond in the diagnostic aggregator, we must
        # send request via the add_diagnostics service. Once a service message is
        # received, the bond will form, and the analyzers will be added to the
        # aggregator.
        rospy.wait_for_service('/diagnostics_agg/add_diagnostics', timeout=10)
        add_diagnostics = rospy.ServiceProxy('/diagnostics_agg/add_diagnostics', AddDiagnostics)

        # The response indicates whether setup was successful or not. If the
        # namespace given is the same as a namespace that was sent previously, the
        # analyzers will not be added.
        resp = add_diagnostics(load_namespace=self.namespace)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def shutdown(self):
        # Make sure the bond is shut down once you're done. If it is not shut down
        # manually, the other side of the bond will automatically shut down after a
        # short time (~5 seconds), removing the analyzers from the aggregator.
        self.bond.shutdown()
        

if __name__ == '__main__':
    m = ManualNode()
    m.start()

```


启动了基本聚合器之后，启动节点
```
roslaunch test_add_analyzers manual_node.launch
```











