## 概述

diagnostic_aggregator包含一个ROS节点，即聚集器节点，用于监听/diagnostics主题上的diagnostic_msgs/diagnosticarray消息，对数据进行处理和分类，并在diagnostics_agg上重新发布。aggregator_node加载“analyzer”插件以执行诊断处理和分类。每个诊断聚合器的配置和设置特定于每个机器人，可以由用户或开发人员确定。

此包中的两个分析器Genericanalyzer和AnalyzerGroup保留并分类诊断消息。它们对于分类组件或系统很有用。它们可以在项目过时时发出警告，但不进行诊断的附加分析。其他分析器可以在启动时由聚合器节点作为插件加载。

聚合的诊断输出可以显示在robot_monitor工具中。robot_监视器以分层格式显示这些消息。要确定层次结构，诊断聚合器会在诊断状态名称前面加上一个"/"路径，例如：

prosilica: Frequency Status →

    My Robot/Sensors/Prosilica/prosilica: Frequency Status
    
### 样例

这显示了聚合器节点如何对简单机器人的诊断数据进行分类和分析。

如果机器人从电机驱动器、SICK 激光扫描仪、摄像机和一些电池进行诊断，则诊断输入可能具有状态名称：

```
Left Wheel
Right Wheel
SICK Frequency
SICK Temperature
SICK Connection Status
Stereo Left Camera
Stereo Right Camera
Stereo Analysis
Stereo Connection Status
Battery 1 Level
Battery 2 Level
Battery 3 Level
Battery 4 Level
Voltage Status
```

使用诊断聚合器，我们可以将这些诊断消息移动到易于理解的类别中，以显示在我们的机器人监控器中。要执行此操作，只需在状态名称前面加上类别名称，然后用“/”分隔它们。

```
My Robot/Wheels/Left
My Robot/Wheels/Right
My Robot/SICK/Frequency
My Robot/SICK/Temperature
My Robot/SICK/Connection Status
My Robot/Stereo/Left Camera
My Robot/Stereo/Right Camera
My Robot/Stereo/Analysis
My Robot/Stereo/Connection Status
My Robot/Power System/Battery 1 Level
My Robot/Power System/Battery 2 Level
My Robot/Power System/Battery 3 Level
My Robot/Power System/Battery 4 Level
My Robot/Power System/Voltage Status
```

使用上述输入，机器人监视器将数据分类如下：

```
My Robot
  -- Wheels
    -- Left
    -- Right
  -- SICK
    -- etc ...
  -- Stereo
  -- Power System
```

## 分析器
 
aggregator_node将加载分析器来存储和处理诊断数据。每个分析器继承自基类diagnostic_aggregator::Analyzer。分析程序必须位于直接依赖PluginLib和diagnostic_aggregator的包中。

基本分析器类是纯虚 diagnostic_aggregator::Analyzer 类。所有派生类都必须实现这些方法： 

* init() - Loads parameters

* match() - Returns true/false if interested in viewing a status message

* analyze() - Analyze a new message

* report() - Report results or state

* getPath() - Get complete path (anything prepended onto status names)

* getName() - Get nice name, like "Motors" or "Sensors" 

分析器可以为他们分析和报告的任何 diagnostic_msgs/DiagnosticStatus消息选择错误状态。通常，分析器的“父级”具有最大子级的错误状态，但某些分析器可能具有更高级的方法。

分析器负责正确设置他们分析的每个项目的名称。聚合器节点不执行任何诊断状态名称层次结构的检查或强制执行。

### 创建一个分析器

用户可以创建分析仪作为不同机器人的插件加载。有关详细信息，请参阅[创建诊断分析器](http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer)的教程。

## 配置聚合器 

有关完整教程，请参见教程[配置Robot的诊断聚合器](http://wiki.ros.org/diagnostics/Tutorials/Configuring%20Diagnostic%20Aggregators)。 

要在机器人上设置诊断分析器，将为聚合器提供配置分析器的参数。下面这些参数位于聚合器节点的专用命名空间中。

```
pub_rate: 1.0 # Optional, defaults to 1.0
base_path: 'PRE' # Optional, defaults to ""
analyzers:
  motors:
    type: PR2MotorsAnalyzer
  joints:
    type: GenericAnalyzer
    path: 'Joints'
    regex: 'Joint*'
```

在~analyzers下的每个子命名空间下，配置了不同的analyzer。分析仪可以是机器人专用的，也可以是通用的，比如GenericAnalyzer。

每个分析器在命名空间中都需要一个特殊的参数type。type参数为aggregator_node提供Analyzer的类名。 

任何未分析的诊断项目都由“其他”分析仪进行分析，并且仍将显示在机器人监视器中。其他“项目或余物在失效5秒后从诊断输出中移除。

pub_rate参数（publish rate of/diagnostics_agg）是可选的，默认为1.0。基本路径预先设置为所有/diagnostics_agg输出，默认为“”（空字符串）。

## 启动诊断聚合器 

要查找正在使用的聚合器的示例，请在“diagnostic_aggregator/demo”目录中查找。用户在聚合器节点的私有参数空间中指定每个分析器，然后启动该节点。

```
<node pkg="diagnostic_aggregator" type="aggregator_node"
      name="diag_agg" >
  <rosparam command="load" 
            file="$(find diagnostic_aggregator)/demo/pr2_analyzers.yaml" />
</node>
```

## 基本分析仪类型

在诊断聚合器包中，提供了两种分析器类型：GenericAnalyzer和AnalyzerGroup。GenericAnalyzer可用于对过时项进行分类和跟踪。分析器组对分析器本身进行分类，允许用户对分析器进行“子分类”。

### GenericAnalyzer

GenericAnalyzer类对于分类诊断数据很有用。它主要用于特定的设备或类别，如Hokuyo节点或Ethercat设备。它可以配置为分析几乎任何一组诊断数据。

有关GenericAnalyzer的完整教程，请参阅[使用GenericAnalyzer](http://wiki.ros.org/diagnostics/Tutorials/Using%20the%20GenericAnalyzer)。

### AnalyzerGroup

AnalyzerGroup可以将一组分析器作为子组加载。这对于分析一组类似的项目很有用，比如4个不同的摄像头。分析组可以使用任何类型的分析仪作为子分析仪之一。

```
type: AnalyzerGroup
path: Sensors
analyzers:
  base_hk:
    type: GenericAnalyzer
    path: Base Hokuyo
    startswith: base_hokuyo_node
    num_items: 3
  tilt_hk:
    type: GenericAnalyzer
    path: Tilt Hokuyo
    startswith: tilt_hokuyo_node
    num_items: 3
  imu:
    type: GenericAnalyzer
    path: IMU
    startswith: imu_node
    num_items: 3
```
上述示例可以分析PR2上的仪器。AnalyzerGroup使用参数路径预先设置所有输出名称。在上面的例子中，

tilt_hokuyo_node: Frequency Status →

    Sensors/Tilt Hokuyo/tilt_hokuyo_node: Frequency Status. 
    
所有子分析器都应位于分析组的~analyzer命名空间下。它们的指定方式应与任何诊断分析器的指定方式相同。

注意：诊断聚合器在内部使用AnalyzerGroup，这就是为什么这与设置诊断聚合器非常相似的原因。

### DiscardAnalyzer

DiscardAnalyzer是GenericAnalyzer的一个子类。它将“匹配”GenericAnalyzer匹配的任何项，但将丢弃这些项而不报告它。这对于从机器人上卸下设备或设置非标准机器人配置非常有用。

### IgnoreAnalyzer

IgnoreAnalyzer只会忽略其命名空间中的所有参数，而不匹配或报告任何内容。

这与上面的DiscardAnalyzer的区别在于，由于IgnoreAnalyzer不匹配任何内容，因此它忽略的任何内容都将在“其他”中报告。DiscardAnalyzer将禁止任何与其匹配的输出。

## ROS API

### aggregator_node
收集诊断，加载诊断分析器插件
#### Subscribed Topics
/diagnostics (diagnostic_msgs/DiagnosticArray)

    Raw diagnostics input 
#### Published Topics
/diagnostics_agg (diagnostic_msgs/DiagnosticArray)

    Processed diagnostics output, at 1.0 Hz (default) 

/diagnostics_toplevel_state (diagnostic_msgs/DiagnosticStatus)

    New in Fuerte: The toplevel status of /diagnostics_agg published with the same rate 
#### Parameters
~pub_rate (double, default: 1.0)

    Rate diagnostics output published 

~base_path (string, default: " " (empty string))

    Prepended to all diagnostics output (ex: "Robot") 

~analyzers ({})

    Load diagnostics analyzers according to these params. Each analyzer is initialized in separate namespace. 
### analyzer_loader
加载诊断分析器，验证初始化是否正常。用作回归测试。
#### Parameters
~analyzers ({})

    Same format as aggregator_node 

## 示例和应用

PR2机器人使用聚合器节点和机器人监视器进行诊断显示。“pr2_bringup/pr2.launch”中的启动文件包含聚合器_节点的配置文件。


“diagnostic_aggregator/demo”目录有一个用于测试和演示的聚合器节点示例。