# pluginlib

pluginlib包提供了使用ROS构建基础设施编写和动态加载插件的工具。为了能够工作，这些工具需要插件提供者在包的package.xml中注册插件。

## 1. 概述

pluginlib是一个C++库用于在ros包中进行加载和卸载插件。插件是动态库中可以被动态加载的类（比如。obj文件，动态链接库文件）。使用pluginlib，我们就不必要明确的将含有这个类的库文件链接到我们的应用程序中。相反，pluginlib可以在任何时候打开一个包含导出类的库，而应用程序不需要事先知道库或包含类定义的头文件。插件对于在不需要应用程序源代码的情况下扩展/修改应用程序行为非常有用。

## 2. 原理

要了解pluginlib的工作原理，让我们考虑一个小例子，首先，假设存在包含多边形基类（“polygon_interface_package”）的ROS 包。也可以说有两种不同类型的多边形的：rectangle_plugin包（矩形）和triangle_plugin包（三角形），rectangle_plugin和triangle_plugin使用都是在package.xml文件中包含指定的export项。这告诉rosbuild构建系统，想在polygon_interface_package包里提供polygon类的插件。增加的export项，事实上是在build/packaging系统里注册这些类。就是说可以通过rospack查询到所有可用的polygon类，它能返回所有可用的类列表，这里主要是rectangle和triangle。

![ros plugin registration](http://wiki.ros.org/pluginlib?action=AttachFile&do=get&target=plugin_model.png)


## 3. 提供插件

### 3.1 注册导出插件

为了允许类被动态加载，它必须被标记为导出类。这是通过特殊宏PLUGINLIB_EXPORT_CLASS来完成的。这个宏可以放在构成插件库的任何源（.cpp）文件中，但通常放在导出类的.cpp文件的末尾。对于上面的示例，我们可能在包'example_pkg'中创建一个class_list.cpp文件。如下所示，并将其编译到librectangle库中：

```
#include <pluginlib/class_list_macros.h>
#include <polygon_interface_package/polygon.h>
#include <rectangle_package/rectangle.h>

//Declare the Rectangle as a Polygon class
PLUGINLIB_EXPORT_CLASS(rectangle_namespace::Rectangle, polygon_namespace::Polygon)
```

### 3.2 插件描述文件

该插件描述文件是用于存储所有关于插件的重要信息的XML文件。它包含有关插件所在的库的信息，插件的名称，插件的类型等。如果我们考虑上面讨论的rectangle_plugin包，插件描述文件（例如rectangle_plugin.xml）将看起来像这样：

```
<library path="lib/librectangle">
  <class type="rectangle_namespace::Rectangle" base_class_type="polygon_namespace::Polygon">
  <description>
  This is a rectangle plugin
  </description>
  </class>
</library>
```

> 我们为什么需要这个文件？

>> 我们需要这个文件除了代码宏，允许ROS系统自动发现，加载和解释插件。

>> 插件描述文件还包含重要信息，如插件的描述，这些信息不适合放在宏里。

### 3.3 注册插件

为了让pluginlib查询跨所有ROS包的系统上的所有可用插件，每个包必须显式指定它导出的插件，以及哪些包库包含这些插件。一个插件提供者必须在其package.xml中的export块指向它的插件描述文件。注意，如果您有其他导出，他们都必须在同一导出字段。rectangle_plugin为例：

```
<export>
  <polygon_interface_package plugin="${prefix}/rectangle_plugin.xml" />
</export>
```

> 重要说明：

>> 为了使上诉export命令正常工作，提供包必须直接依赖于包含插件接口的包

>> 例如，rectangle_plugin必须在器catkin/package.xml中具有以下行：

```
<build_depend>polygon_interface_package</build_depend>
<run_depend>polygon_interface_package</run_depend>

```

### 3.4 查询插件

可以通过rospack查询ROS包系统，以查看任何给定包可用的插件。例如：

```
rospack plugins --attrib=plugin nav_core
```

这将返回从nav_core包导出的所有插件

### 3.5 使用插件

pluginlib在class_loader.h头文件中提供了一个ClassLoader类，使得它能够快速和容易地使用提供的类。
下面，我们将展示一个使用ClassLoader在一些使用多边形的代码中创建矩形实例的简单示例：

```
#include <pluginlib/class_loader.h>
#include <polygon_interface_package/polygon.h>

//... some code ...

pluginlib::ClassLoader<polygon_namespace::Polygon> poly_loader("polygon_interface_package", "polygon_namespace::Polygon");

try
{
  boost::shared_ptr<polygon_namespace::Polygon> poly = poly_loader.createInstance("rectangle_namespace::Rectangle");

  //... use the polygon, boost::shared_ptr will automatically delete memory when it goes out of scope
}
catch(pluginlib::PluginlibException& ex)
{
  //handle the class failing to load
  ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
}
```

重要说明：

> 在使用插件时，ClassLoader不能超出范围。

> 所以，如果你在类中加载一个插件对象，请确保类加载器是该类的成员变量。