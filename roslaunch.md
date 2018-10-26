#gprof/Callgrind + Kcachegrind  
  以上两个工具可以对程序框架，运行情况进行分析，具体使用方式[How to profile roslaunch nodes](http://wiki.ros.org/roslaunch/Tutorials/Profiling%20roslaunch%20nodes)  
  gprof会以特定的参数进行编译和链接，在过程中会在项目函数中插入分析语句（"mcount"or"_mcount"or"__mcount"）之后在程序运行时会自动生成分析数据，  
  通过使用gprof命令即可将分析文件进行打印出来进行阅读，以上提到的两种工具均可以在roslaunch中以配置方式进行使用。可以帮助我们进行分析项目中存在的效率痛点，以及整体的调用关系。  
#命令参数使用

  roslaunch可以直接以launch文件文件，或者包名+空格+launch文件名使用。
  
  -p 指定rosmaster端口，--wait等待直到有roscore启动，--local 只启动本地节点忽略远程节点，--screen 强制所有的打印到屏幕， -v版本信息，--dump-params 打印所有的参数值
  
  roslaunch my_file.launch arg:=value  直接在命令行传递参数
  
  --nodes 查看该roslaunch启动的所有节点名字； --args 查看某一个节点的传入参数； --find 查看某个节点所在的文件，大型launch文件中查找方便； --files 定位节点所在的launch文件
  
  --ros-args 打印出所有的参数信息，并进行提示。 roslaunch-deps列出启动整个项目所需要的所有依赖包； roslaunch-check检查launch文件合法性； roslaunch-logs和cd命令搭配使用直接定位日志位置
  具体使用方式可以直接查看[Commandline Tools](http://wiki.ros.org/roslaunch/Commandline%20Tools#roslaunch-check)
  
#rqt_launch/rqt_launchtree

	roslaunch在rqt的gui插件中提供了launch的功能，可以按照一个launch文件中所有的节点进行全部开启，关闭，或者独立节点的开启关闭或者重启。
	
	rqt_launchtree可以进行节点关系的梳理，将每个launch文件中的include展开，以及每个launch文件中的节点进行递归展示

#roslaunch中的调试方法

  在roslaunch中<node>标签内使用launch-prefix属性可以添加该节点的调试仿真方式，基本用法有如下几种：
##launch-prefix="xterm -e gdb --args"
  roslaunch启动之后会为该节点启动一个独立的终端窗体进入gdb调试状态，使用gdb的run命令即可运行程序，同样可以打断电以及单步方针，即就是一个完整的gdb调试环境。
  
##launch-prefix="gdb -ex run --args
  在同一个终端窗体内以gdb的方式启动该节点，并不需要手动执行run命令，该节点就进入运行状态。
  
##launch-prefix="valgrind"
  在valgrind中运行该节点
  
##launch-prefix="xterm -e"
  在一个独立的终端窗体内运行该节点
  
##launch-prefix="nice"
  以低功耗的方式运行该节点，也就是意味着低效率
  
##launch-prefix="screen -d -m gdb --args"
  该属性可以很好的支持远程仿真调试，当以该方式启动远程节点的时候，可以通过ssh登录到远程主机，使用命令： screen -D -R来启动查看gdb对话窗口
  
##launch-prefix="xterm -e python -m pdb"
  在一个独立的窗体内运行pdb来调试Python代码，需要手动运行run命令进行仿真调试
  
##launch-prefix="yappi -b -f pstat -o <filename>"
  以多线程的方式运行rospy，  例如：yappi
  
##launch-prefix="/path/to/run_tmux"
在一个新的tmux窗口内运行该节点，需要手工创建一个/path/to/run_tmux文件如下：
    ```
  #!/bin/sh
  tmux new-window "gdb --args $*"
    ```
#api usage
  roslaunch的api是不稳定的，wiki明确说明并不保证不变化，但是可以保证launch文件内的标签属性是向后兼容的。
  
  目前只提供了基于python的roslaunch调用接口，可以启动一个ros节点，也可以启动一个roslaunch文件。如果需要在C++里面调用，可以考虑C++与python的混合编程。如果只是简单的
  
  启动一个launch文件，那么可以考虑C++的系统调用等方式启动一个脚本。
  
  具体的api调用方式可以参考wiki中：[API Usage](http://wiki.ros.org/roslaunch/API%20Usage)
  
#format XML
  单线程处理，深度优先遍历include文件，串行化读取所以会有以最后设置生效。
##参数替换方式

###$(env ENVIRONMENT_VARIABLE)
从当前环境变量中进行提取变量，如果环境变量未设置该值，那么为未设置。同事用env设置的变量不可以被改变。

###$(optenv ENVIRONMENT_VARIABLE) $(optenv ENVIRONMENT_VARIABLE default_value)
从当前环境变量中提取该变量值，如果设置了default_value那么当环境变量不存在时会设置default_value，否则为空字符串。

###$(find pkg)
指定一个包相关的文件或者目录，为了解决硬编码带来的一些问题，例如：`$(find rospy)/manifest.xml`

###$(anon name)
多用于生成一个匿名的节点名

###$(arg foo)
将<arg>标签定义的值进行提取，要求必须有一个对应的<arg>标签定义在该launch文件中，例如：
```
<node name="add_two_ints_server" pkg="beginner_tutorials" type="add_two_ints_server" />
<node name="add_two_ints_client" pkg="beginner_tutorials" type="add_two_ints_client" args="$(arg a) $(arg b)" />
roslaunch beginner_tutorials launch_file.launch a:=1 b:=5
```

###$(eval <expression>)
可以计算python表达式

###$(dirname)
用于获取当前launch文件所在文件夹的绝对路径名，可以结合其他一些变量方式进行使用

##if and unless
1 ‘true’------->true
0 'false' ------> false
其他值会导致错误
用法：
```
	<group if="$(arg foo)">
	  <!-- stuff that will only be evaluated if foo is true -->
	</group>
	
	<param name="foo" value="bar" unless="$(arg foo)" />  <!-- This param won't be set when "unless" condition is met -->
```

##标签

###<launch>
是一个launch文件的根元素，作为其他角色的容器存在。
该容器内可以包含如下成员：
<node> 启动一个节点；<param> 设置参数服务器上参数；<remap> 生命重新映射一个名字；<machine> 声明一个用于launch的机器；
<rosparam>通过rosparam文件来设置ros参数；<include> 包含一个其他的roslaunch文件；<env> 指定节点的环境变量；<test> 启动一个test节点；
<arg> 声明一个参数；<group> 组内作为一个命名空间，或者统一进行重映设；

###<node>
指定一个节点用于启动

####属性
#####pkg="mypackage"
节点所在的包

#####type="nodetype"
节点对应的可执行文件名

#####name="nodename"
节点名，不可以包括命名空间，必须使用 ns属性来设置命名空间

#####args="arg1 arg2 arg3"(可选)
节点参数

#####machine="machine-name"(optional, see <machine>)
在指定的机器上运行该节点

#####respawn="true"(optional, default: False)
当节点挂掉的时候自动重启该节点

#####respawn_delay="30" (optional, default 0) New in ROS indigo
如果自动启动生效，那么当节点挂掉之后等待respawn_delay秒再尝试重新启动

#####required="true"(optional)
如果该节点不存在，那么整个roslaunch的启动都将被取消

#####ns="foo"(optional)
设置节点的命名空间

#####clear_params="true|false"(optional)
在启动该节点之前，从该节点的私有命名空间里面清空已有的参数

#####output="log|screen"(optional)
该节点的日志文件打印位置，log为日志文件，screen为屏幕， stderr为屏幕
    
#####cwd="ROS_HOME|node"(optional)
节点运行的工作空间，如果为ROS_HOME那么为整个ros的工作空间，如果为node，那么为该节点所在文件系统的位置。

#####launch-prefix="prefix arguments"(optional)
启动该节点的前置，来生命启动方式： gdb, valgrind, xterm, nice, or other handy tools. 

####成员

<env> 为该节点设置一个环境变量；<remap> 为该节点的参数设置重映设；
<rosparam> 加载一个参数文件到该节点的本地命名空间中；<param> 在该节点的本地空间中设置一个参数

###<machine>
如果只在本地机器上运行节点不需要设置该标签，它主要是用来设置远程主机的ssh和ros环境变量的，当然也可以设置本机的ros环境变量。
具体详细情况可以查看wiki：[machine](http://wiki.ros.org/roslaunch/XML/machine)

####属性
#####name="machine-name"
机器名字，与<node>中，machine属性一致

#####address="blah.willowgarage.com"
网络可达的一个机器地址或者主机名

#####env-loader="/opt/ros/fuerte/env.sh" New in Fuerte
指定远程主机上的环境变量设置脚本位置

#####default="true|false|never" (optional)
设置这个机器为默认机器来部署节点，如果没有设置默认主机，那么本地主机作为该机器

#####user="username" (optional)
ssh用户名，如果不需要可以省略

#####password="passwhat"(strongly discouraged)
ssh密码，强烈建议设置免密登录
#####timeout="10.0" (optional)
秒，设置登录远程机器启动程序时的超时时间

####成员
<env> 为在这个机器上运行的所有节点设置环境变量

###<include>
用于将其他的launch文件包含到当前launch文件中，但是master节点只会在第一个文件中启动一次。

####属性
#####file="$(find pkg-name)/path/filename.xml"
将要包含进来的launch文件

#####ns="foo" (optional)
包含进来以后，被包含进来的launch文件中节点的命名空间

#####clear_params="true|false" (optional Default: false)
删除被包含launch文件中的所有的参数，要求命名空间必须被指定
 
####成员
<env> 为被包含进来的所有节点设置环境变量；<arg>为被包含进来的文件设置参数

###<remap>
为ros节点提供一个名字重映设的功能，多用于参数名的重映设。，作用范围包括，<launch>,<group>,<node>

####属性

#####from="original-name"
待重映设的名字

#####to="new-name"
将要被映射为的名字

###<env>
为ros节点提供一个环境变量值，作用范围包括，<launch>,<include>,<node>,<machine>标签。当在launch标签中使用的时候，
其生效范围只会影响到该定义之后的所有节点。

####属性

#####name="environment-variable-name"
即将要设置的环境变量名字
    
#####value="environment-variable-value"
即将要设置的环境变量的值

###<param>
用来给参数服务器定义参数和值，并不只是可以值的形式进行定义，也可以textfile和binfile的形式，该标签可以在<node>内部使用，不过那就会成为该节点私有的参数设置。

####属性

#####name="namespace/name"
参数的命名空间和名字，应该尽量避免全局命名
    
#####value="value"(optional)
定义该参数变量的值，如果这个这个属性被忽略，那么必须指定textfile，或者binfile，或者command

#####type="str|int|double|bool|yaml"(optional)
指定参数的类型，如果不明确指定，那么roslaunch会自动判断后给出类型。规则是：
包含小数点的数字为float，反之为整数；    
true 和 false为boolean类型  
剩余的全部都定义为字符串  

#####textfile="$(find pkg-name)/path/file.txt"(optional)
指定textfile的文件位置，推荐使用pkg-name的方式进行定位

#####binfile="$(find pkg-name)/path/file"(optional)
指定binfile的文件位置，推荐使用pkg-name的方式进行定位

#####command="$(find pkg-name)/exe '$(find pkg-name)/arg.txt'"(optional)
指定命令作为参数

#####为了使用yaml文件，应该如下方式进行使用：
`<rosparam command="load" file="FILENAME" />`

###<rosparam>
用来指定一个yaml文件来作为参数，并将文件中定义的值同步到参数服务器中  
删除（delete）和复制（dump）命令应该在加载（load）命令之前出现并执行  
<rosparam>标记可以引用YAML文件或包含原始YAML文本。如果YAML文本定义了字典，则可以省略param属性。

####属性

#####command="load|dump|delete" (optional, default=load)
rosparam命令
#####file="$(find pkg-name)/path/foo.yaml" (load or dump commands)
配置文件名

#####param="param-name"
参数名字

#####ns="namespace" (optional)
指定命名空间 

#####subst_value=true|false (optional)
允许在YAML文本中使用替换参数。

###<group>
<group>标记可以更轻松地将设置应用于一组节点。它具有ns属性，允许您将节点组推送到单独的命名空间。您还可以使用<remap>标记在整个组中应用重映射设置

####属性

#####ns="namespace" (optional)
将节点组分配给指定的命名空间。命名空间可以是全局的或相对的，但不鼓励使用全局命名空间。

#####clear_params="true|false" (optional)
在启动之前删除组名称空间中的所有参数。此功能非常危险，应谨慎使用。必须指定ns。

####元素
<node> 待启动节点；<param> 设置参数服务器上的参数；<remap> 重映设参数名字；  
<machine>机器信息；<rosparam>加载配置文件；<include>包含其他的launch文件；  
<env>设置环境变量；<test>启动一个测试节点；<arg>定义一个参数


###<test>
<test>标记在语法上与<node>标记类似。它们都指定要运行的ROS节点，但<test>标记表示该节点实际上是要运行的测试节点。有关这些测试节点的更多信息，请参阅rostest文档。

####一个例子
```
<test test-name="test_1_2" pkg="mypkg" type="test_1_2.py" time-limit="10.0" args="--test1 --test2" />
```

####属性
#####<test>标签共享大多数正常的<node>属性，除了：
    没有respawn属性（测试节点必须终止，因此它们不可重新生成）  
    没有输出属性，因为测试使用自己的输出记录机制  
    机器属性被忽略  
#####必须有的属性

######pkg="mypackage"
测试节点所在的包

######test-name="test_name"
在测试结果中记录时使用的该测试节点的名字

######type=“nodetype”
是该测试节点的可执行文件的文件名

#####可选的属性
######name="nodename"
节点名称。注意：name不能包含命名空间。请改用ns属性。如果未指定此属性，则test-name的值将用作节点名称

######args="arg1 arg2 arg3"
给节点传的参数

######clear_params="true|false"
在启动之前删除节点的私有命名空间中的所有参数。

######cwd="ROS_HOME|node"
如果是“node”，则节点的工作目录将设置为与节点的可执行文件相同的目录。在C Turtle中，默认为'ROS_HOME'。  
在boxturtle（ROS 1.0.x）中，默认为'ros-root'。在cturtle中不推荐使用'ros-root'。

######launch-prefix="prefix arguments"
用于预先添加到节点的启动参数的命令/参数。这是一个强大的功能，使您可以启用gdb，valgrind，xterm，nice或其他方便的工具。有关示例，请参阅Valgrind或GDB中的Roslaunch节点。

######ns="foo"
设置命名空间

######retry="0"
在将测试视为失败之前重试测试的次数。默认值为0.此选项对于有时会出现故障的随机过程非常有用。

######time-limit="60.0"
测试被视为失败之前的秒数。默认值为60秒

####元素
<env>为节点设置环境变量;<remap>为此节点设置重映射参数。;  
<rosparam>将rosparam文件加载到此节点的〜/ local命名空间中;<param>在节点的〜/ local命名空间中设置参数

###<arg>
<arg>标签允许您通过指定通过命令行传递的值，通过<include>传入或声明为更高级别的文件来创建更多可重用和可配置的启动文件。   
Args不是全局性的。 arg声明特定于单个启动文件，非常类似于方法中的本地参数。您必须将arg值显式传递给包含的文件，就像在方法调用中一样。

####<arg>的三种用法

#####<arg name="foo" />
声明foo的存在。 foo必须作为命令行参数（如果是顶级）或通过<include>传递（如果包含）传递。

#####<arg name="foo" default="1" />
使用默认值声明foo。 foo可以通过命令行参数（如果是顶级）或通过<include>传递（如果包含）覆盖。

#####<arg name="foo" value="bar" />
用常量值声明foo。无法覆盖foo的值。此用法启用启动文件的内部参数化，而不会在较高级别公开该参数化。

####属性

#####name="arg_name"
参数名

#####default="default value" (optional)
参数的默认值。不能与value属性结合使用。
 
#####value="value" (optional)
参数的值。不能与default_value属性结合使用。

#####doc="description for this arg" (optional) New in Indigo
参数描述。


##实例









