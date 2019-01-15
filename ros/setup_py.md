## setup.py

如果您的ROS包中包含要安装的python模块和脚本，则需要定义安装过程以及使脚本在develspace中可访问的方法。python生态系统在distuils或setuputils库中定义安装标准。使用这些库，包将在项目根目录中名为setup.py的文件中定义安装文件。py文件使用python描述堆栈的python内容。
我们建议使用distutils包而不是setuptools/distribute，因为使用distutils可以避免在项目源文件夹中创建egg info文件夹。catkin不支持distuils2的setup.cfg文件。
catkin允许您在此setup.py中指定python文件的安装，并重用cmakelists.txt中的一些信息。
您可以通过包括行来完成此操作： 

>catkin_python_setup()  添加到CmakeLists.txt文件中

catkin将使用热补丁版本distutils执行set up.py以读取设置devel空间的参数，并使用适当的参数执行set up.py以安装到cmake_install_前缀下的catkin安装空间。
这意味着您不应使用以下命令执行setup.py：
```
# DO NOT USE
# python setup.py install
```

手动安装，因为这将安装到不同的位置，并且您将拥有多个相互影响的已安装版本。使用setup.py创建catkin包的pypi包目前不支持ROS消息和服务，并且pypi上没有核心的ROS库（例如ros py），因此使用setup.py创建pypi对ROS节点不是很有用。
对于develspace，catkin将使用setup（）的以下setup.py参数：
```$xslt
from distutils.core import setup

setup(
    version='...',
    scripts=['bin/myscript'],
    packages=['mypkg'],
    package_dir={'': 'src'}
)

```

这将为脚本中列出的所有脚本创建中继到devel空间中的一个文件夹，在该文件夹中可以找到并执行脚本，还可以为包中列出的任何包中继包。中继包是一个文件夹，其中包含一个uu init_uuuy文件夹，而没有其他文件夹。在python中导入此文件夹将执行uuu init_uuy.py的内容，然后使用python exec（）函数将原始python模块导入源空间中的文件夹。

该版本将与package.xml中声明的版本进行比较，并在不匹配时引发错误。

## Using package.xml in setup.py



使用catkin_pkg便利功能，可以在不复制package.xml中包含的信息的情况下编写setup.py文件，如下所示： 
```$xslt
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mypkg'],
    scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
```


这将解析package.xml并对字段进行格式化，以便在一个作者分发给pypi的情况下，可以为setup.py很好地设置多个带电子邮件的作者。 



>ROS用户通常不应使用scripts参数，就像在ROS中一样，可执行文件应该使用rosrun执行，而不是安装到全局bin文件夹中。安装此类python脚本的一种方法是将以下内容添加到cmakelists.txt：
```$xslt
install(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
``` 


## genmsg交互


genmsg是一个为ROS消息提供语言绑定的外部catkin包。使用genmsg宏时，存在排序约束，在这种情况下，必须按以下顺序调用宏：

```cmake
project(...)
...
find_package(catkin ...)
...
catkin_python_setup()
...
generate_messages()
...
catkin_package()
```