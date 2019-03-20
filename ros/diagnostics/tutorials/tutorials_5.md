使用GenericAnalyzer分析来自EtherCAT设备、硬件驱动程序和机器人计算机的诊断信息。显示了GenericAnalyzer的完整特性集。

## 概述

diagnostics _aggregator包用于收集、处理和分析关于/诊断主题的数据。为此，它使用aggregator_node加载“诊断分析器”插件。GenericAnalyzer是这些插件中最基本的一个，可以配置为在任何机器人系统上使用。

GenericAnalyzer可以很容易地在一个公共头下对一组诊断数据进行分类。当使用robot_monitor查看时，数据将出现在树中的公共父项下。

例如，hokuyo_node可能使用以下名称发布诊断数据:

```
tilt_hokuyo_node: Connection Status
tilt_hokuyo_node: Driver Status
tilt_hokuyo_node: Frequency Status
```

使用hokuyo_node的GenericAnalyzer可能会收集这些数据并使用名称输出诊断:
```
Tilt Hokuyo
Tilt Hokuyo/Connection Status
Tilt Hokuyo/Driver Status
Tilt Hokuyo/Frequency Status
```
当在robot_monitor中查看时，“Tilt Hokuyo”项下面会有“Connection Status”等项。如果任何子项进入警告或错误状态，“Tilt Hokuyo”项也将进入该状态。

GenericAnalyzer不仅使诊断更容易查看，还可以在条目过期、丢失或进入警告或错误状态时发出警告。

其他教程，比如配置诊断聚合器教程，展示了诊断聚合器如何加载分析程序。本教程展示了如何使用GenericAnalyzer的所有特性。

### GenericAnalyzer 代码
“diagnostic_aggregator / include / diagnostic_aggregator / generic_analyzer。声明GenericAnalyzer类，这是最基本的分析程序。aggregator_node使用它来存储、处理和重新发布诊断数据。GenericAnalyzer由pluginlib作为分析器插件加载。
### 加载GenericAnalyzer
要将GenericAnalyzer加载到aggregator_node中，GenericAnalyzer参数必须位于aggregator_node的~analyzers名称空间中。~analyzers下的每个名称空间将创建一个分析器。
```
analyzers:
  motors:
    type: GenericAnalyzer
    path: Motors
    contains: 'motor'
```

看看上面的参数:

* **motor**:每个分析器加载在~analyzers下的新名称空间中。在本例中，是motors名称空间。

* **type**:所有分析程序，包括GenericAnalyzer，都使用类型参数作为插件加载分析程序。类型参数是Analyzer插件的类名。

* **contains**:在本例中，GenericAnalyzer将分析包含“motor”的任何值。除了下面包含的选项外，我们还将解释用于匹配诊断数据的选项。

有关将诊断分析器加载到aggregator_node的详细信息，请参见配置诊断聚合器。

## 分析设备驱动程序
让我们分析来自hokuyo_node的诊断，如上面的示例所示。从示例输入中可以看到，所有传入数据的名称都以“tilt_hokuyo_node”开头。可以设置GenericAnalyzer来收集以某个前缀开始的任何数据。

将其添加到aggregator_node的YAML文件中:
```
analyzers:
  tilt_hokuyo:
    type: GenericAnalyzer
    path: Tilt Hokuyo
    startswith: tilt_hokuyo_node
```

现在，任何名称以“tilt_hokuyo_node”开头的诊断项都将在“Tilt Hokuyo”下收集。

不幸的是，这仍然在每个状态名前面留下难看的“tilt_hokuyo_node”前缀。由于所有数据都将在“Tilt Hokuyo”路径下，所以没有必要这样做。要去掉这个，请添加:

```
    remove_prefix: tilt_hokuyo_node
```

直到文件末尾。

缩写:

```
    find_and_remove_prefix: tilt_hokuyo_node
```

我们还需要对Hokuyo分析仪做一件事。如果Hokuyo在启动后崩溃，我们将不会在诊断中看到它。我们可以向GenericAnalyzer添加一个num_items参数，以确保我们拥有准确数量的诊断项。

倾斜Hokuyo分析仪参数变为:

```
analyzers:
  tilt_hokuyo:
    type: GenericAnalyzer
    path: Tilt Hokuyo
    find_and_remove_prefix: tilt_hokuyo_node
    num_items: 3
```
现在，我们将监视来自“tilt_hokuyo_node”的所有内容，清理处理过的名称，如果我们没有从节点获得任何数据，分析程序将报告缺少项。

## 分析了一个PR2电源系统

以下参数将创建一个GenericAnalyzer来监视PR2电源系统。

```
analyzers:
  powersystem:
    type: GenericAnalyzer
    prefix: Power System
    expected: [ 
      'IBPS 0',
      'IBPS 1']
    startswith: [
      'Smart Battery']
    name: [
      'Power Node 1018']
    contains: [
      'Battery']
    timeout: 10
```

timeout参数告诉GenericAnalyzer将超时中没有更新的任何项标记为“陈旧的”。它将在robot_monitor中显示一个特殊的“陈旧”图标。默认为5.0秒。将其用于电力系统非常重要，因为电池驱动程序并不总是快速更新。
## 分析控制器

PR2控制器控制机器人的运动。当机器人由pr2_controller_manager操作时，控制器被加载和卸载。

```
analyzers:
  controllers:
    type: GenericAnalyzer
    path: Controllers
    name: [
      'pr2_base_controller',
      'torso_trajectory_controller' ]
    expected: 'Realtime Control Loop'
    regex: 'control*'
    discard_stale: true
```

* **name** -- name参数需要精确的名称匹配。可以是字符串或列表。

* **expected**--期望的项目需要精确的名称匹配。如果它不存在，将被报告为陈旧和丢失。可以是字符串或列表。

* **regex**—用户可能更愿意使用正则表达式来配置他们的分析器。使用regex参数来给出regex值。可以是字符串或列表。

* **discard_stale**—由于控制器可以由pr2_controller_manager卸载，如果没有收到控制器的消息，我们不想报告处于“陈腐”状态的控制器。超时期间未更新的任何项都将被丢弃。由于“实时控制循环”是“预期的”，所以我们从不丢弃它。

## 更多技巧

### 顶层状态
GenericAnalyzer报告一个名为“path”的“顶级”状态。在上面的控制器示例中，顶层名称是“controllers”。顶层的状态级别(OK/Warning/Error/陈腐)由子元素决定。

* 所有子元素的最大值-顶级状态是所有子元素的最大值。孩子受到警告，父母也会受到警告。

* 陈旧的子元素表示“错误”——如果任何子元素陈旧，则顶级状态将为“错误”，除非……

* 所有陈旧的子节点都是“陈旧的”——如果所有的子节点都是陈旧的，那么顶层状态将是“陈旧的”，并且有消息“All陈腐的”
### 超时
timeout参数用于将项标记为过期。任何消失超过超时时间的项都被标记为过期，并将在robot_monitor中显示一个过期的图标。带有一个或多个陈旧项的GenericAnalyzer的顶级状态将有一个错误状态。默认超时时间为5秒。小于零的超时意味着诊断数据永远不会被标记为过期。
### 格式化参数
下面的配置参数可以作为字符串或字符串列表给出。

* startswith

* contains

* name

* expected

* remove_prefix

* find_and_remove_prefix

* regex

























