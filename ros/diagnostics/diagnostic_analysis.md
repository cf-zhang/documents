诊断分析包可以将诊断数据日志转换为一系列csv文件。robot日志是用rosbag记录的，可以使用此包中的脚本脱机处理。

## 概述

使用Rosrecord和Rosplay等ROS工具，用户可以录制和播放包含ROS消息的包文件。对于诊断包文件，我们使用diagnostic_analysis包中的工具将这些包文件转换为csv文件。export_csv.py工具将这些包文件转换为csv文件，每个诊断_msgs/diagnosticstatus名称都有一个单独的csv文件。这些csv文件可以使用现成的电子表格软件进行分析。

对于通用电子表格软件（最多支持65536行），较大的包文件可能太大。对于大型文件，sparse_csv.py可以精简这些文件或将其转换为较小的csv文件进行分析。


### 非诊断程序包文件 

如果您有任何不在“诊断”主题中的消息，它们将被丢弃。

```
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor
```

pr2诊断工具链使用“/diagnostics”主题上的"diagnostic_msgs/diagnosticarray”消息，这是此包的设计目的。只有这种类型的包文件才能与诊断分析包一起使用。要播放其他包文件，请使用Rostopic。

## 命令行工具

### export_csv.py

export_csv.py [bag-files]...

将包文件转换为csv表示.默认情况下，这会将csv文件输出到output/log_文件名目录，例如：

```
$ rosrun diagnostic_analysis export_csv.py <bagfilename>
```

-d OUTPUT_DIR

更改输出目录。例如： 
```
rosrun diagnostic_analysis export_csv.py file.bag -d my_dir
```
将结果存储在“my_dir/output”目录中。 

诊断状态消息中的每个“名称”都有自己的文件。这意味着您将看到一个电源板文件，另一个Ethercat主机文件，等等…

### sparse_csv.py

sparse_csv.py <csv-file> -m (--max) 

将csv文件缩小到最大65000行。这有助于在65536行的Excel限制内进行拟合。这将保留每N行，以减少csv行长度。例如：
```
$ rosrun diagnostic_analysis sparse_csv.py my_bagfile.csv -m
```
sparse_csv.py <csv-file> -l LENGTH (--length)

    根据需要跳过每N行，将csv文件缩小到指定的行长度。 
    
sparse_csv.py <csv-file> -s SKIP (--skip) 

    通过跳过每个跳过行来收缩csv文件。 
    
## Tutorials

[离线分析诊断日志 ](http://wiki.ros.org/diagnostics/Tutorials/Analyzing_Diagnostic_Logs)

本教程使用diagnostic_analysis来分析robot日志文件。这些文件由机器人在运行时自动生成，并包含机器人的整个诊断历史。您可以将这些日志文件转换为可以使用任何电子表格编辑器打开的csv文件（逗号分隔值）。这仅适用于pr2诊断工具链中使用的/diagnostics主题上的diagnostic msgs/diagnosticarray消息。