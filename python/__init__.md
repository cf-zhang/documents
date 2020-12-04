<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](https://github.com/thlorenz/doctoc)*

- [__init__.py的作用](#__init__py%E7%9A%84%E4%BD%9C%E7%94%A8)
  - [标识该目录是一个python的模块包（module package）](#%E6%A0%87%E8%AF%86%E8%AF%A5%E7%9B%AE%E5%BD%95%E6%98%AF%E4%B8%80%E4%B8%AApython%E7%9A%84%E6%A8%A1%E5%9D%97%E5%8C%85module-package)
  - [简化模块导入操作](#%E7%AE%80%E5%8C%96%E6%A8%A1%E5%9D%97%E5%AF%BC%E5%85%A5%E6%93%8D%E4%BD%9C)
    - [__init__.py 是怎么工作的？](#__init__py-%E6%98%AF%E6%80%8E%E4%B9%88%E5%B7%A5%E4%BD%9C%E7%9A%84)
    - [控制模块导入](#%E6%8E%A7%E5%88%B6%E6%A8%A1%E5%9D%97%E5%AF%BC%E5%85%A5)
    - [偷懒的导入方法](#%E5%81%B7%E6%87%92%E7%9A%84%E5%AF%BC%E5%85%A5%E6%96%B9%E6%B3%95)
  - [配置模块的初始化操作](#%E9%85%8D%E7%BD%AE%E6%A8%A1%E5%9D%97%E7%9A%84%E5%88%9D%E5%A7%8B%E5%8C%96%E6%93%8D%E4%BD%9C)
  - [利用__init__.py](#%E5%88%A9%E7%94%A8__init__py)
  - [__init__.py的设计原则](#__init__py%E7%9A%84%E8%AE%BE%E8%AE%A1%E5%8E%9F%E5%88%99)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# __init__.py的作用
我们经常在python的模块目录中会看到 "__init__.py"  这个文件，那么它到底有什么作用呢？
## 标识该目录是一个python的模块包（module package）
如果你是使用python的相关IDE来进行开发，那么如果目录中存在该文件，该目录就会被识别为 module package 。

## 简化模块导入操作

假设我们的模块包的目录结构如下：

```
.
└── mypackage
    ├── subpackage_1
    │   ├── test11.py
    │   └── test12.py
    ├── subpackage_2
    │   ├── test21.py
    │   └── test22.py
    └── subpackage_3
        ├── test31.py
        └── test32.py
```

如果我们使用最直接的导入方式，将整个文件拷贝到工程目录下，然后直接导入：

```
from mypackage.subpackage_1 import test11
from mypackage.subpackage_1 import test12
from mypackage.subpackage_2 import test21
from mypackage.subpackage_2 import test22
from mypackage.subpackage_3 import test31
from mypackage.subpackage_3 import test32
```

当然这个例子里面文件比较少，如果模块比较大，目录比较深的话，可能自己都记不清该如何导入。（很有可能，哪怕只想导入一个模块都要在目录中找很久）

这种情况下，__init__.py 就很有作用了。我们先来看看该文件是如何工作的。

### __init__.py 是怎么工作的？
实际上，如果目录中包含了 __init__.py 时，当用 import 导入该目录时，会执行 __init__.py 里面的代码。

我们在mypackage目录下增加一个 __init__.py 文件来做一个实验：

```
.
└── mypackage
    ├── __init__.py
    ├── subpackage_1
    │   ├── test11.py
    │   └── test12.py
    ├── subpackage_2
    │   ├── test21.py
    │   └── test22.py
    └── subpackage_3
        ├── test31.py
        └── test32.py
```

mypackage/__init__.py 里面加一个print，如果执行了该文件就会输出：

```print("You have imported mypackage")```

下面直接用交互模式进行 import 

```
>>> import mypackage
You have imported mypackage
```
很显然，__init__.py 在包被导入时会被执行。

### 控制模块导入
我们再做一个实验，在 mypackage/__init__.py 添加以下语句：

```
from subpackage_1 import test11
```

我们导入 mypackage 试试:
```
>>> import mypackage
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/taopeng/Workspace/Test/mypackage/__init__.py", line 2, in <module>
    from subpackage_1 import test11
ImportError: No module named 'subpackage_1'
```

报错了。。。怎么回事？

原来，在我们执行import时，当前目录是不会变的（就算是执行子目录的文件），还是需要完整的包名。

```
from mypackage.subpackage_1 import test11
```

综上，我们可以在__init__.py 指定默认需要导入的模块　

### 偷懒的导入方法

有时候我们在做导入时会偷懒，将包中的所有内容导入

```
from mypackage import *
```

这是怎么实现的呢？ __all__ 变量就是干这个工作的。

__all__ 关联了一个模块列表，当执行 from xx import * 时，就会导入列表中的模块。我们将 __init__.py 修改为 。

```
__all__ = ['subpackage_1', 'subpackage_2']
```

这里没有包含 subpackage_3，是为了证明 __all__ 起作用了，而不是导入了所有子目录。

```
>>> from mypackage import *
>>> dir()
['__builtins__', '__doc__', '__loader__', '__name__', '__package__', '__spec__', 'subpackage_1', 'subpackage_2']
>>> 
>>> dir(subpackage_1)
['__doc__', '__loader__', '__name__', '__package__', '__path__', '__spec__']
```
子目录的中的模块没有导入！！！

该例子中的导入等价于

```
from mypackage import subpackage_1, subpackage_2
```

因此，导入操作会继续查找 subpackage_1 和 subpackage_2 中的 __init__.py 并执行。（但是此时不会执行 import *）

我们在 subpackage_1 下添加 __init__.py 文件:

```
__all__ = ['test11', 'test12']

# 默认只导入test11
from mypackage.subpackage_1 import test11
```

再来导入试试

```
>>> from mypackage import *
>>> dir()
['__builtins__', '__doc__', '__loader__', '__name__', '__package__', '__spec__', 'subpackage_1', 'subpackage_2']
>>> 
>>> dir(subpackage_1)
['__all__', '__builtins__', '__cached__', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__path__', '__spec__', 'test11']
```
如果想要导入子包的所有模块，则需要更精确指定。

```
>>> from mypackage.subpackage_1 import *
>>> dir()
['__builtins__', '__doc__', '__loader__', '__name__', '__package__', '__spec__', 'test11', 'test12']
```

## 配置模块的初始化操作
在了解了 __init__.py 的工作原理后，应该能理解该文件就是一个正常的python代码文件。

因此可以将初始化代码放入该文件中。

## 利用__init__.py

```
#
# @file __init__.py
#

import arithmetic.add
import arithmetic.sub
import arithmetic.mul
import arithmetic.dev

add = arithmetic.add.add
sub = arithmetic.sub.sub
mul = arithmetic.mul.mul
dev = arithmetic.dev.dev
```

在__init__.py中， 我们import了arithmetic下的所有子模块，并在__init__.py中给各个子模块的核心功能取了新的名字，作为arithmetic模块的变量。所以我们在main.py中import了arithmetic模块之后，就可以直接进行使用了。如果你使用from arithmetic import * 语句，那么我们就可以使用add、sub、mul、dev，连a4都省了。

## __init__.py的设计原则

__init__.py的原始使命是声明一个模块，所以它可以是一个空文件。在__init__.py中声明的所有类型和变量，就是其代表的模块的类型和变量，第2小节就是利用这个原理，为四则运算的4个子模块声明了新的变量。我们在利用__init__.py时，应该遵循如下几个原则：

+ 不要污染现有的命名空间。模块一个目的，是为了避免命名冲突，如果你在种用__init__.py时违背这个原则，是反其道而为之，就没有必要使用模块了。

+ 利用__init__.py对外提供类型、变量和接口，对用户隐藏各个子模块的实现。一个模块的实现可能非常复杂，你需要用很多个文件，甚至很多子模块来实现，但用户可能只需要知道一个类型和接口。就像我们的arithmetic例子中，用户只需要知道四则运算有add、sub、mul、dev四个接口，却并不需要知道它们是怎么实现的，也不想去了解arithmetic中是如何组织各个子模块的。由于各个子模块的实现有可能非常复杂，而对外提供的类型和接口有可能非常的简单，我们就可以通过这个方式来对用户隐藏实现，同时提供非常方便的使用。

+ 只在__init__.py中导入有必要的内容，不要做没必要的运算。像我们的例子，import arithmetic语句会执行__ini__.py中的所有代码。如果我们在__init__.py中做太多事情，每次import都会有额外的运算，会造成没有必要的开销。一句话，__init__.py只是为了达到B中所表述的目的，其它事情就不要做啦。


