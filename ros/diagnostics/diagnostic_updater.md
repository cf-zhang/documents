diagnostic_updater包含易于更新诊断的工具。它通常用于设备驱动程序，以跟踪输出主题的状态、设备状态等。

## 诊断更新程序API 

diagnostic_updater提供各种C++实用工具，以帮助将诊断与软件集成。常见的更新程序任务包括：

* 在设备驱动程序上发布传感器数据主题的状态

* 报告硬件设备已关闭

* 当值超出界限时报告错误，例如温度

它还提供了非常类似于C++ API的Python API。

 ### 示例和教程
 
诊断更新程序的工作示例可以在[diagnostic_updater/src/example.cpp]( http://docs.ros.org/api/diagnostic_updater/html/example_8cpp_source.html)中找到。这个例子介绍了更新程序的一些最常见的用法。 
 
 ### API稳定性
 
文档化的C++ API被广泛使用，应该被认为是稳定的。 

### diagnostic_updater::DiagnosticStatusWrapper

The diagnostic_updater::DiagnosticStatusWrapper类减轻了填写diagnostic_msgs/DiagnosticStatus消息的痛苦。它处理设置摘要，可能使用消息字段的打印类型格式，以及使用类型转换和值格式设置键值对。它还具有合并多个诊断/诊断状态消息的功能，而不会丢失状态消息中的信息。

### diagnostic_updater::Updater

diagnostic_updater::Updater类管理一组诊断更新函数及其定期发布。

### diagnostic_updater::DiagnosedPublisher

 可以使用预打包的诊断更新功能来监视主题的频率、常见的诊断功能及其时间戳的有效性。diagnostic_updater::DiagnosedPublisher 将与之相关的发布服务器和标准诊断程序包装在一个类中。