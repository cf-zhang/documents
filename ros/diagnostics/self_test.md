## 运行一个self-test

您可以使用rosservice命令获取可用子测试的列表： 

```
$ rosservice list | grep "/self_test$"
/hokuyo_node/self_test
```

然后可以使用rosservice运行自检： 
```
$ rosservice call /hokuyo_node/self_test
```

您也可以使用不推荐使用的运行自测节点获得不同的输出： 
```
$ rosrun self_test run_selftest /hokuyo_node/self_test
```

## C++ API

self_test:：testranner类将公布一个服务“~ self_test”。调用时，它将检查节点的连接状态，以及开发人员希望在节点或设备上执行的任何其他检查。



自测试包的API的一个示例用法可以在 [self_test/src/selftest_example.cpp.](https://github.com/ros/diagnostics/blob/hydro-devel/self_test/src/selftest_example.cpp)

### 稳定性和状态

文档里的C++和ROS API应该被认为是稳定的。 

### self_test::TestRunner

Self_Test包含Self_Test:：TestRunner类，可用于对要运行的一组测试进行排序以测试设备。它为自助测试服务做广告。调用服务时，Self_Test:：TestRunner调用已按顺序定义的测试，并将结果组合到诊断_msgs/DiagnosticStatus数组中（请参见服务定义诊断_msgs/SelfTest）。一个详细的示例[self_test/src/self test_example.cpp](https://github.com/ros/diagnostics/blob/hydro-devel/self_test/src/selftest_example.cpp)。 

## Nodes

### selftest_rostest
运行自检并在自检通过时报告成功的花名册。

#### Parameters
~node_to_test (string, default: "")

    要在其上运行自检的节点的名称。 

~max_delay (string, default: 60)

    等待自我测试服务出现的最长时间。