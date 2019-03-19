## 概述

诊断系统的设计目的是从硬件驱动程序和机器人硬件收集信息，供用户和操作员进行分析、故障排除和记录。诊断堆栈包含用于收集、发布、分析和查看诊断数据的工具。

诊断工具链是围绕/diagnostics主题构建的。在本主题中，硬件驱动程序和设备将发布带有设备名称、状态和特定数据点的诊断程序/诊断阵列消息。

diagnostic_updater和self_test包允许节点收集和发布诊断数据。diagnostic_aggregator可以在运行时对诊断进行分类和分析。操作员和开发人员可以使用rqt_robot_monitor包查看诊断数据。diagnostic_analysis包可以将诊断日志转换为csv文件以供检查和事实分析后使用。

##  收集和发布

要收集和发布诊断数据，节点可以使用diagnostic_updater包中的工具。更新程序允许节点（尤其是硬件驱动程序）收集和发布诊断数据。使用更新包中的工具，驱动程序可以监视频率和连接状态。self-test包使用诊断更新程序使用特殊服务调用对驱动程序执行自测。

## 分析和汇总 

diagnostic_aggregator包包含用于在运行时分类和分析诊断的工具。使用插件模型，可以为不同类型的机器人配置诊断聚合器，以便对诊断进行简单分析。很容易将数百个诊断项目归纳为几个类别。诊断聚合插件允许开发人员为用户提供易于理解的常见问题消息。

## 查看

rqt_robot_monitor包包含robot_monitor工具，该工具以图形形式显示诊断_聚合器中处理的数据。

对于没有diagnostic_aggregator的机器人，rqt_runtime_monitor包包含一个简单的监视器，显示来自/diagnostics主题的数据。

## 离线分析

diagnostic_analysis中的工具允许用户将诊断包文件转换为一个或多个csv文件，以便使用现成的电子表格软件进行绘图或查看。export_csv.py工具允许将包文件转换为多个csv文件，并将每个诊断状态名称转换为自己的文件。

## 更多信息

更多信息见:[Diagnostic System for Robots Running ROS. ](http://www.ros.org/reps/rep-0107.html)


[diagnostic_aggregator](diagnostic_aggregator.md)

[diagnostic_analysis](diagnostic_analysis.md)

[diagnostic_common_diagnostics](diagnostic_common_diagnostics.md)

[diagnostic_updater](diagnostic_updater.md)

[self_test](self_test.md)