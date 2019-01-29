# class_loader

class_loader包是一个ROS独立的包，用于在运行时加载插件，并且是高级ROS“pluginlib”库的基础。class_loader利用主机操作系统的运行时装入器打开运行时库（例如.so/.dll文件）。内省所导出插件类的库，并允许用户实例化所述导出类的对象，而不必为这些类显式声明（即头文件）。

## 1. 概述

class_loader是一个ROS的独立包，它用于在运行时从一个动态库里（例如，.so/.dll文件）动态的加载被导出的C++类，以及创建这些类的对象。通过类加载器加载的类与仅针对运行时库进行链接并使用其中的类不同的是，您的代码不需要在客户机代码中定义类（即类的头文件）。这种加载类的方式通常被成为插件

### 1.1 class_loader Vs pluginlib

class_loader用于高级ROS包pluginlib的实现，这是在ROS生态系统中装入插件的鼓励方法。当创建面向非ROS包的插件时，应该使用class_loader。当将插件导出到ROS包时，应该使用pluginlib。
                                                           
## 2. 使用方法

类装入器使用简单，需要与单个库（libclass_loader）链接。 

### 2.1 接口

接口是通过两个类提供的，分别是class_loader::ClassLoader和class_loader::MultiLibraryClassLoader。两者都提供类似的接口，但前者只绑定到一个运行时库，而后者可以与多个库关联。典型的工作流程如下：

* 在源文件中包含 class_loader/class_loader.h

* 通过路径和名字初始化一个class_loader::ClassLoader对象以打开这个库文件

```
 class_loader::ClassLoader loader("libMyLibrary.so");
```

*查询类以查找具有由某个基类定义的接口的导出类（示例中为MyBase）

```
std::vector<std::string> classes = loader.getAvailableClasses<MyBase>()
```

* 创建/销毁所述导出类的对象

```
 for(unsigned int c = 0; c < classes.size(); ++c)
 {
   boost::shared_ptr<MyBase> plugin = loader.createInstance<MyBase>(classes[c]);
   plugin->someMethod();
   //'plugin' will automatically be deleted when it goes out of scope
 }
``` 

*销毁ClassLoader对象以关闭库。

### 2.2 示例：ClassLoader的基本工作流

```
#include <class_loader/class_loader.h>
#include "MyBase.h" //Defines class MyBase

int main()
{
  class_loader::ClassLoader loader("libMyLibrary.so");
  std::vector<std::string> classes = loader.getAvailableClasses<MyBase>();
  for(unsigned int c = 0; c < classes.size(); ++c)
  {
    boost::shared_ptr<MyBase> plugin = loader.createInstance<MyBase>(classes[c]);
    plugin->someMethod();
  }
}
```

### 2.3 使类可导出并成为公共基类

需要注意的是，如果这些类注册为可导出类，则类加载器只能检查类并创建这些类的对象。 对于要导出的任何类，请确保对源（.cpp）文件中的每个类都声明以下宏： 

```
CLASS_LOADER_REGISTER_CLASS(Derived, Base)
``` 

 其中，Derived是指要导出的类的名称，而Base是指派生该类的类的名称。 虽然您不需要在将加载类的代码中定义派生类，但您仍然需要对其基类进行定义，以便能够使用插件。 注意，在代码示例中，当通过getAvailableClasses（）内省库或通过createInstance（）创建插件时，这两个方法都需要一个指示基类的模板类型参数。如果没有给出正确的基类，类加载器将看不到该类或该类不可实例化。可以注册具有不同基类的类，甚至可以在同一个库中使用不同的基多次注册相同的类，并且可以传递地注册相同的类加载器。请注意，在编译时必须通过模板参数为重要的方法提供基类参数。使用宏生成所有源文件后，可以将对象文件打包到运行时库中，然后通过类加载器加载该库。
 
### 2.4 线程安全以及类型校验
 
 所有的class_loader::ClassLoader和class_loader::MultiLibraryClassLoader的所有方法都是线程安全的。模板的使用允许对所有类型进行静态验证，并且不可能加载具有不兼容接口的无效插件。这是一个强有力的概念，可以保证插件在编译和运行时的完整性，当然这在机器人学中非常重要。
 
### 2.5 了解装载和卸载

当使用类加载器创建和销毁插件时，会导致打开和关闭包含这些插件的运行库。当操作系统加载运行库时，主机可执行文件中作为运行库存根的符号将被解析。卸载库时，由于代码已从可执行文件的地址空间中删除，并且[可能]已从内存中卸载，因此这些相应内存地址处的符号不可用。如果试图使用未解析的符号，则会导致运行时链接错误。这意味着，如果我从运行时库中定义的类中创建一个对象，然后卸载该库，然后尝试使用该对象……事情会变糟。

#### 2.5.1 loadLibrary() and unloadLibrary()

在上面的示例代码中，库是由class_loader自动加载和卸载的，尽管class_loader：：ClassLoader提供了显式加载和卸载底层库的方法（分别为loadLibrary（）和unloadLibrary（））。class_loader很聪明，因为它会在创建时自动加载库，并在类加载器超出范围时卸载库。人们可能想知道为什么加载和卸载方法会暴露出来。为了理解这一点，我们必须首先了解class_loader的按需（惰性）加载/卸载模式。

#### 2.5.2 按需（惰性）库加载/卸载





























 