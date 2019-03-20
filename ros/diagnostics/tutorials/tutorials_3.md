本教程使用diagnostics _analysis包分析机器人日志文件。这些文件由机器人在运行时自动生成，包含机器人的整个诊断历史。您可以将这些日志文件转换为CSV文件(逗号分隔值)，这些文件可以用任何电子表格编辑器打开。这只适用于PR2诊断工具链中使用的关于/diagnostics主题的diagnostics _msgs/ diagnostics array消息。
## 日志文件的类型
如果您有任何不在“/diagnostics”主题上的消息，它们将被丢弃。

```
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor
Discarding message on topic: /turtle1/pose
Discarding message on topic: /turtle1/color_sensor

```
PR2诊断工具链在“/diagnostics”主题上使用diagnostics _msgs/ diagnostics array消息，这正是这个包的设计目的。只有这种类型的包文件才能使用diagnostics stic_analysis包。要播放其他包文件，请使用rostopic。
## 恢复日志文件

要从机器人的日志文件中恢复数据，请查看pr2上的/hwlog文件夹。日志文件以“.bag”结尾。，并直接用机器人的诊断数据记录下来。
```
$ cd /hwlog
```

文件使用“prX_runtime_automatic_YYYY-MM-DD-HH-MM-SS-topic.bag”标签存储。

您可能想要将这些复制到机器人的目录中。例如，如果你的电脑是xxx，而你的用户名是user:

```
$ scp prX_runtime_automatic_YYYY-MM-DD-HH-MM-SS-topic.bag user@xxx:<somewhere_you_want_it_path>
```
如果你要找的日志文件不在那里，检查一下文件夹:

```
stor6:/prdata/bag_repository
```
日志文件在夜间同步期间被移动到那里。

## 将日志文件转换为csv文件
机器人日志文件可以查看与普通电子表格软件处理后与CSV导出。当在一个包文件上运行时，这会在output/log_filename目录中创建几个CSV文件。

```
$ rosrun diagnostic_analysis export_csv.py <bagfilename>
```
您可以使用“-d”选项来更改输出目录。例如
```
rosrun diagnostic_analysis export_csv.py <bagfilename> -d <dir>
```
将结果存储在“/output”目录中。

诊断状态消息中的每个“名称”都有自己的文件。这意味着您将看到一个用于电源板的文件，另一个用于EtherCAT Master，等等。

## 给日志文件瘦身
有些包文件太大，无法用普通的电子表格软件查看。在Excel 2007之前，Office和Excel版本的打开限制为65,536行。如果要进行长时间的分析，只保留每一行n，这可能有助于使CSV变得“稀疏”。

为此，使用sparse_css .py。

```
$ rosrun diagnostic_analysis sparse_csv.py <path/my_bagfile>.csv -m
```
生成得文件<path/my_bagfile>_sparse.csv ，最多65000行。

## 分析

OpenOffice Calc 是一个很好的工具,例如
```
$ oocalc output/BAG_DIRECTORY/<my_bagfile>.csv
```
在OpenOffice Calc中打开数据。从那里，您可以分析和绘制数据。










