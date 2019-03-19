此包提供用于监视Linux主机的通用节点。

## Nodes
 
### hd_monitor.py
 
 通过hddtemp守护进程监视硬盘状态。这将使用Smart检查每个驱动器的温度，并检查驱动器是否未被卸下或拔下。
 
#### Published Topics
 
diagnostics (diagnostic_msgs/DiagnosticArray)

    硬盘诊断信息
    
### ntp_monitor.py

监视计算机之间的NTP同步状态。将要连接的主机名指定为第一个命令行参数，并且可以使用其他命令行参数设置偏移公差。有关更多选项，请参阅rosrun diagnostic_common_diagnostics ntp_monitor.py-h。

#### Published Topics

diagnostics (diagnostic_msgs/DiagnosticArray)

    NTP diagnostic information 
    
### sensors_monitor.py

使用sensors命令监视系统传感器。这将使用系统的默认阈值检查系统的内置温度、电压和风扇转速状态。

#### Published Topics

diagnostics (diagnostic_msgs/DiagnosticArray)

    Sensor diagnostic information 
    
#### Parameters

~ignore_fans (bool, default: False)

    Ignore fan speed warnings 
    
### tf_monitor.py

监视TF状态。检查是否正在发布tf消息、转换是否是最新的、tf状态中是否存在循环、每个转换是否只有一个权限发布，以及帧是否具有一致的父级。

#### Subscribed Topics
tf (tf/tfMessage)

    TF topic 

#### Published Topics
diagnostics (diagnostic_msgs/DiagnosticArray)

    TF diagnostic information 