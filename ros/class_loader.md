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

ClassLoader构造函数提供了一个可选的布尔标志（OnDemand_LoadUnload），用于指示ClassLoader是否根据需要执行库的按需（延迟）加载，并在销毁由它创建的最后一个插件后自动关闭它。默认情况下，此标志设置为假。当设置为false时，ClassLoader在构建时加载库，并在销毁时卸载库。在按需模式下，仅当通过createInstance（）创建第一个插件时才打开库，当ClassLoader创建的唯一剩余插件被销毁时才卸载库。当我们希望最小化内存中加载的库的数量时，此模式非常有用。

#### 2.5.3 加载/卸载调用计数

控制库的加载和卸载时间通常很有用，特别是在按需模式下。可以通过调用LoadLibrary来强制执行此操作。 当调用loadlibary和unloadlibrary时，库被强制加载到内存中，在调用unloadlibrary之前，系统无法自动卸载。这允许多个线程共享同一个类加载器来强制共享库，即使它是在按需模式下打开的，也可以在内存中保留到完成为止。这不是线程安全问题，而是一个性能问题，如果一个线程在一段时间内非常需要库，并且希望保证库保留在内存中，那么可以防止库不断地被加载/卸载。

#### 2.5.4 托管对象与非托管对象

class_loader::ClassLoader允许以boost：：shared_ptr值的形式创建对象，以便自动清理对象。实现此功能的另一个重要原因是，用户不能过早卸载库。用户可以通过备用class_loader::Classloader::CreateUnmanagedInstance方法自由创建非托管对象，但这将阻止ClassLoader在内存中仍有对象时停止用户卸载库。 这是因为类加载器无法知道非托管实例的状态。

>建议使用shared_ptr版本class_loader::Classloader::CreateInstance（），以确保类的安全使用。

>警告

>> 如果同时创建托管实例，将class_loader置于惰性模式并创建非托管实例是非常危险的！

### 2.6 异常错误处理

如果发生错误，许多ClassLoader方法将引发基类类型为Class_Loader::ClassLoaderException的异常。class_loader::classloaderexception的子类在class_loader/class_loader_exceptions.h中定义，并指出各种问题。这些包括：   

* class_loader::LibraryLoadException - 无法加载一个库文件

* class_loader::LibraryUnloadException - 无法卸载一个库文件

* class_loader::CreateClassException - 无法创建指定类的对象

### 2.7 多库类加载器

ClassLoader设计为仅绑定到单个运行时库。通常，打开多个库很方便，并且能够通过统一的接口从中加载/卸载类。这也就是为什么提供了class_loader::MultiLibraryClassLoader。这个类提供了一个和class_loader::ClassLoader几乎相同的接口，但允许将多个库绑定到一个加载器对象。 在内部，class_loader::MultiLibraryClassLoader只是类class_loader::ClassLoader对象集合的管理器。

### 2.8 小心直接链接到插件库

从1.9版开始的pluginlib和class_loader非常不鼓励将应用程序直接链接到包含插件的库。通常情况下，用户会将插件放在库中，并沿着要直接链接的代码放置插件。其他时候，他们希望能够将类作为插件使用，也可以在没有类加载器的情况下直接使用它们。这在以前的pluginlib版本中是很好的，但是作为1.9版本，pluginlib位于class_loader程序的顶部，用于插件加载，而插件加载程序无法处理这个问题。

#### 2.8.1 问题是孤儿阶级的工厂。     

问题是类装入器实现插件内省的方式是让插件类工厂在库打开时自动注册自己。 这不是1.9版之前的pluginlib的工作方式。但是，问题在于，当您直接将可执行文件链接到带有插件的库时，当该程序启动时，所有插件工厂都将在class_loader::ClassLoader的范围之外创建。

#### 2.8.2 向后兼容警告

class_loader可以补偿并仍然运行，这样遗留代码就可以正常工作。 然而，根本的问题是工厂过早地被加载，class_loader不知道这些工厂来自何处，并且没有class_loader绑定到它们。 如果试图从直接链接到的库中创建插件，则class_loader本质上是猜测它所拥有的工厂是与所需类对应的工厂。

> 名称冲突 

>> 但是，如果您有两个定义同一类的不同库，则会出现问题。这可能导致名称冲突，并且无法判断哪个工厂来自哪个库。在这种情况下，类装入器将生成一条警告消息。

>无法卸载插件库

>> 同样，class_loader不能再正确关闭库，因为它不能判断客户端可执行文件是否仍在使用同一库中的非插件代码。试图过早关闭库将导致程序崩溃，因此未完成。通常情况下，这不是一个问题，因为操作系统会在程序关闭时进行清理，但对于真正需要关闭库、取消链接并在执行时从内存中删除的程序，您不能也不能直接链接。

#### 2.8.3 示例：多库类加载器的基本工作流

```
#include <class_loader/multi_library_class_loader.h>
#include "MyBase.h" //Defines class MyBase

int main()
{
  class_loader::MultiLibraryClassLoader loader;
  loader.loadLibrary("libSomeLib.so");
  loader.loadLibrary("libAnotherLib.so");
  std::vector<std::string> classes = loader.getAvailableClasses<MyBase>();
  for(unsigned int c = 0; c < classes.size(); ++c)
  {
    boost::shared_ptr plugin<MyBase> = loader.createInstance<MyBase>(classes[c]);
    plugin->someMethod();
  }
}
```

## 3.设计细节

### 3.1 设计和实现

虽然class_loader包的实现非常简单，但是代码在内部可能有点难以理解。因此，本文给出了一些设计注意事项，以指导今后的维护人员。

#### 3.1.1 动机

ROS从一开始就通过pluginlib包实现了插件的概念。尽管pluginlib足以加载插件，但它决定重构代码以解决某些缺陷，并使其更易于维护。以下是pluginlib的一些缺点

* 对ROS构建系统的依赖性意味着想要使用pluginlib但不使用ROS包管理系统的应用程序不能这样做。

* 对于从任意运行库导出的插件，没有真正的内省。自省通过ROS构建系统和XML文件的内容提供。

* 不是线程安全的

* 它使用的是第三方Poco库的一个黑客版本和未记录版本，并作为基线代码的一部分包含在内。

最终的解决方案是将pluginlib分为两个包：一个类加载器来实现类的加载/卸载（class_loader），以及位于它上面的现有pluginlib。PluginLib本身不会改变API的方式，这样就不会依赖于它来破坏包。 

#### 3.1.2 依赖POCO

和pluginlib一样class_loader依赖于开源Poco库，以提供从运行库加载和卸载类的底层能力。与使用修改过的poco版本的传统pluginlib不同，class_loader使用stock版本（libpoco dev）。另外，class_loader不使用Poco的类装入器类，而是使用较低级别的Poco:：SharedLibrary类，以便提供比Poco:：ClassLoader提供的类注册技术更方便的类注册技术。

#### 3.1.3 类图

下面是class_loader的类图。注意，并不是所有的类都显示出来，只有公共接口是公开的，并且一些真正由const&传递的方法参数为了紧凑性似乎是按值传递的。

![](http://wiki.ros.org/class_loader?action=AttachFile&do=get&target=class_loader_classdiagram.png) 

#### 3.1.4 类注册的理解

当一个库被加载进内存的时候，宏 CLASS_LOADER_REGISTER_CLASS 是用来向ClassLoader注册类的。这个机制就是类加载器如何能够对库进行内省。用于打开库的底层库poco有自己的类加载器poco::class loader，它执行的功能与class_loader::class loader几乎相同。区别在于注册的工作方式。

##### 3.1.4.1 是什么导致一个插件系统难于设计

理解为什么实现一个插件系统是困难的是很有帮助的。以下是一些原因：

* C++是原生的，编译的（没有动态的eval()函数，如Lisp或Python)，以及静态类型的语言（需要知道编译时的插件接口的类型信息），没有任何可自省的运行时（例如Java JVM，Python VM）……这是一个困难的问题。

* 当您加载库（例如，通过Linux上的dlopen（））时，您不能检查库中是否有符号！您可以在命令行上执行一个NM，解析它，解单符号，向用户显示生成的函数名和签名。在Linux上，我们找不到本地的方法来完成这项工作。您可以对nm.c进行黑客攻击，使其成为一个函数……或者从命令行调用nm。此外，您只能调用具有编译时函数签名的C++代码中的函数，因此调用任意函数而不调用C++编译器听起来很困难

* 即使你提前知道符号，符号也会因为C++而被破坏，而不是跨平台和编译器的标准。

##### 3.1.4.2 C++静态变量实例化自动注册

在poco:：classloader中，用户必须在一个源文件中的一个位置注册所有类。这可能有点不方便，所以类装入器在此基础上进行了改进。相反，用户可以在每个类中注册，然后在不同的库中混合和匹配类，而不用担心。CLASS_LOADER_REGISTER_MACRO的工作方式是，它展开为新的结构类型和相同类型的相应静态全局变量。此类的构造函数调用：

```
class_loader::class_loader_private::registerPlugin<Base,Derived>();
```

会创建一个 class_loader::class_loader_private::MetaObject<Base,Derived>类型的工厂。这些工厂由插件系统存储，用于通过其create（）方法创建插件。

这个技巧的作用在于，当库加载时，C++标准规定全局变量首先被实例化。这将调用静态变量的构造函数，然后调用上面的注册函数。这意味着每个注册的类都会自动添加到全局可用的列表中。

##### 3.1.4.3 全局函数怎么了？

class_loader的核心是在命名空间pulgins::plugins_private中的一组全局函数内实现的。最初的想法是在一个类中创建内容，但是由于实现问题，恢复了必须使所有数据结构都全局可用，并且拥有一个类是没有意义的。更清晰的结果只是一组全局函数，并在class_loader:：Classloader和class_loader:：MultilibraryClassloader类中写入公开的接口。

 



















 