## 初始化和关闭

### 1. 初始化

#### 1.1 配置PythonPath

setup.py

#### 1.2 初始化节点

rospy.init_node(name, anonymous=False, log_level=rospy.INFO, disable_signals=False)

#### 1.3 命令行参数

remapping arguments

### 2. 关闭

#### 2.1 两种等待关闭的方法

```$xslt
while not rospy.is_shutdown():
   do some work
```

```$xslt
... setup callbacks
rospy.spin()
```

#### 2.2 注册关闭时的钩子

rospy.on_shutdown(h)

```$xslt
def myhook():
  print "shutdown time!"

rospy.on_shutdown(myhook)
```

#### 2.3 手动关闭节点

rospy.signal_shutdown(reason)

## 消息

### 1. 生成消息

```$xslt
package_name/msg/Foo.msg → package_name.msg.Foo
package_name/srv/Bar.srv → package_name.srv.Bar
```
回生成在src目录下

### 2.消息初始化

```$xslt
msg = std_msgs.msg.String()
msg.data = "hello world"
```

```$xslt
msg = std_msgs.msg.ColorRGBA(255.0, 255.0, 255.0, 128.0)
```

```$xslt
msg = std_msgs.msg.ColorRGBA(b=255)
```

## 发布与订阅

### 1.发布一个话题

```$xslt
pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
pub.publish(std_msgs.msg.String("foo"))
```

#### 1.1 发布者初始化

```$xslt
rospy.Publisher(topic_name, msg_class, queue_size)
```

#### 1.2 Publisher.publish()

```$xslt
pub.publish(std_msgs.msg.String("hello world"))
```

```$xslt
pub.publish("hello world")
```

```$xslt
pub.publish(b=255)
```

#### 1.3 queue_size: 发布行为和排队

如果发布速度超过了Rospy通过网络发送消息的速度，Rospy将开始删除旧消息

#### 1.4 选择一个好的queue_size

消息队列缓冲区大小设置的太大或者太小都不好

##### 1.4.1 忽略大小值

如果省略关键字参数，则不会传递任何参数，或者对于groovy和较旧的ROS分发版，将同步处理发布。由于Indigo未传递关键字参数，队列大小将导致向控制台打印警告。 

##### 1.4.2 None

不推荐。发布是同步处理的，这意味着一个阻止订阅服务器将阻止所有发布。从靛蓝通过时，没有会导致警告打印到控制台。

##### 1.4.3 Zero

虽然值0表示无限队列，但这可能很危险，因为内存使用量可能无限增长，因此不建议使用。 

##### 1.4.4 one, two, three

如果您的系统没有过载，您可能会争辩说，一条排队的消息应该在十分之一秒内被调度程序线程接收。因此，当使用10赫兹时，1/2/3的队列大小是绝对可以的。

如果要确保新发布的值将始终阻止删除任何旧的尚未发送的值，则将队列大小设置为1是一种有效的方法。比如说，这对于只关心最新测量结果的传感器是有好处的。例如，如果存在较新的测量值，则不要发送较旧的测量值。 

##### 1.4.5 ten or more

当使用大型队列（如10或更大）时，用户界面消息（如Digital_IO，一种按钮状态）就是一个示例，它可以从大型队列中获益，以防止丢失值的更改。另一个例子是，当您希望记录所有已发布的值时，包括那些在以高速率/小队列大小发布时将丢弃的值。

#### 1.5 完整示例

```$xslt
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   pub.publish("hello world")
   r.sleep()

```

### 2.订阅一个topic

```$xslt
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
```

#### 2.1 连接信息
```$xslt
print m._connection_header
{'callerid': '/talker_38321_1284999593611',
 'latching': '0',
 'md5sum': '992ce8a1687cec8c8bd883ec73ca41d1',
 'message_definition': 'string data\n\n',
 'topic': '/chatter',
 'type': 'std_msgs/String'}


```

## 服务

### 1. 服务定义，请求消息，反馈消息

### 2. 服务代理

```$xslt
rospy.wait_for_service('add_two_ints')
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
try:
  resp1 = add_two_ints(x, y)
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))
```

####2.1 调用方式

```$xslt
req = rospy_tutorials.srv.AddTwoIntsRequest(1, 2)
resp = add_two_ints(req)
```

```$xslt
resp = add_two_ints(1, 2)
```

```$xslt
resp = add_two_ints(a=1)
```

####2.2 长连接

### 3. 提供服务

```$xslt
  def add_two_ints(req):
      return rospy_tutorials.srv.AddTwoIntsResponse(req.a + req.b)

  def add_two_ints_server():
      rospy.init_node('add_two_ints_server')
      s = rospy.Service('add_two_ints', rospy_tutorials.srv.AddTwoInts, add_two_ints)
      rospy.spin()
```

回调函数编写方式
```$xslt
def add_two_ints(req):
  return [req.a + req.b]
```

```$xslt
def add_two_ints(req):
  return req.a + req.b
```

```$xslt
def add_two_ints(req):
  return {'sum': req.a + req.b}
```

#### 3.1 关闭

stop:
```$xslt
s = rospy.Service('add_two_ints', rospy_tutorials.srv.AddTwoInts, add_two_ints)
...
s.shutdown('shutdown reason')

```

wait for:
```$xslt
s = rospy.Service('add_two_ints', rospy_tutorials.srv.AddTwoInts, add_two_ints)
s.spin() #returns when either service or node is shutdown
```

### 4. 服务连接头

client：
```$xslt
h = { 'cookies' : 'peanut butter' }
s = rospy.ServiceProxy('foo', Foo, headers=h)

```

server：
```$xslt
  def add_two_ints(req):
      who = req._connection_header['callerid']
      if 'cookies' in req._connection_header:
          cookies = req._connection_header['cookies']
      return AddTwoIntsResponse(req.a + req.b)

  def add_two_ints_server():
      rospy.init_node(NAME)
      s = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
```

## 参数服务器

不是线程安全的

### 1. 获取参数

```$xslt
global_name = rospy.get_param("/global_name")
relative_name = rospy.get_param("relative_name")
private_param = rospy.get_param('~private_name')
default_param = rospy.get_param('default_param', 'default_value')

# fetch a group (dictionary) of parameters
gains = rospy.get_param('gains')
p, i, d = gains['p'], gains['i'], gains['d']
```

### 2.设置参数

```$xslt
# Using rospy and raw python objects
rospy.set_param('a_string', 'baz')
rospy.set_param('~private_int', 2)
rospy.set_param('list_of_floats', [1., 2., 3., 4.])
rospy.set_param('bool_True', True)
rospy.set_param('gains', {'p': 1, 'i': 2, 'd': 3})

# Using rosparam and yaml strings
rosparam.set_param('a_string', 'baz')
rosparam.set_param('~private_int', '2')
rosparam.set_param('list_of_floats', "[1., 2., 3., 4.]")
rosparam.set_param('bool_True', "true")
rosparam.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")

rospy.get_param('gains/p') #should return 1

```

### 3.参数存在

```$xslt
if rospy.has_param('to_delete'):
    rospy.delete_param('to_delete')
```

### 4.删除参数
```$xslt
try:
    rospy.delete_param('to_delete')
except KeyError:
    print "value not set"
```


###5.查找参数路径

```$xslt
param_name = rospy.search_param('global_example')
v = rospy.get_param(param_name)
```

###6.获取参数名字s

```$xslt
rospy.get_param_names()
```

## 日志

```$xslt
rospy.logdebug(msg, *args, **kwargs)
rospy.loginfo(msg, *args, **kwargs)
rospy.logwarn(msg, *args, **kwargs)
rospy.logerr(msg, *args, **kwargs)
rospy.logfatal(msg, *args, **kwargs)
```

### 1.读取日志

![日志显示关系图](document/image/readinglog.png)

### 2.样例

```$xslt
    topic = 'chatter'
    pub = rospy.Publisher(topic, String)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("I will publish to the topic %s", topic)
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rospy.sleep(0.1)
```

### 3.周期性日志
```$xslt
while True:
    rospy.loginfo_throttle(60, "This message will print every 60 seconds")

```

### 4.一次性日志
```$xslt
while True:
    rospy.loginfo_once("This message will print only once")
```
### 5.高级日志配置

### 6.日志等级UI
rqt_logger_level 

## 名字和节点信息

### 1. 获取节点信息

rospy.get_name()

    Get the fully-qualified name of this node 

rospy.get_namespace()

    Get the namespace of this node 

rospy.get_node_uri()

    Get the XMLRPC URI of this node 
    
### 2. 修改名字

rospy.resolve_name(name, caller_id=None)


## 时间

### 1.时刻与时间

```$xslt
int32 secs
int32 nsecs
```

#### 1.1 获取当前时间
rospy.Time.now(), rospy.get_rostime()

##### 1.1.1 时间0
当使用模拟时钟时间时，get_rostime（）返回时间0，直到第一条消息在/clock上收到为止，因此0实质上意味着客户机还不知道时钟时间。因此，应以不同的方式处理值0，例如循环访问get_rostime（），直到返回非零。

#### 1.2 创建时间实例rospy.Time(secs=0, nsecs=0)

```$xslt
epoch = rospy.Time() # secs=nsecs=0
t = rospy.Time(10) # t.secs=10
t = rospy.Time(12345, 6789)
```

```$xslt
t = rospy.Time.from_sec(123456.789)
```

#### 1.3 时刻与时间的转换

```$xslt
t = rospy.Time.from_sec(time.time())
seconds = t.to_sec() #floating point
nanoseconds = t.to_nsec()

d = rospy.Duration.from_sec(60.1)  # One minute and one tenth of a second
seconds = d.to_sec() #floating point
nanoseconds = d.to_nsec()
```

#### 1.4 时间计算

```$xslt
two_hours = rospy.Duration(60*60) + rospy.Duration(60*60)
one_hour = rospy.Duration(2*60*60) - rospy.Duration(60*60)
tomorrow = rospy.Time.now() + rospy.Duration(24*60*60)
negative_one_day = rospy.Time.now() - tomorrow
```

### 2.休眠与频率

rospy.sleep(duration) 

```$xslt
# sleep for 10 seconds
rospy.sleep(10.)

# sleep for duration
d = rospy.Duration(10, 0)
rospy.sleep(d)
```

rospy.Rate(hz)

```$xslt
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    pub.publish("hello")
    r.sleep()
```

### 3. 定时器
rospy.Timer(period, callback, oneshot=False)

```$xslt
def my_callback(event):
    print 'Timer called at ' + str(event.current_real)

rospy.Timer(rospy.Duration(2), my_callback)

```

停止这个定时器 shutdown()

## 异常

ROSException

    Base exception class for ROS clients 

ROSSerializationException

    Exception for message serialization errors 

ROSInitException

    Exception for errors initializing ROS state 

ROSInterruptException

    Exception for operations that interrupted. This is most commonly used with rospy.sleep() and rospy.Rate 

ROSInternalException

    Base class for exceptions that occur due to internal rospy errors (i.e. bugs). 

ServiceException

    Errors related to communicate with ROS Services