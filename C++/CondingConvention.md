# 灵动科技(Forwardx)编码规范(C++)
本文档对灵动科技C++开发编码进行约束，加强代码一致性，使得代码易于阅读、理解以及修改，从而使得代码易于管理(依据[Google 开源项目风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/))。文档仅对编码规范进行要求，未详细说明原因。若需详细了解，请参考链接[Google 开源项目风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/)

本编码规范的编写原则：对自己提供的代码负责、为他人着想、美观。  

文档结构：
1. 头文件
2. 作用域
3. 类
4. 命名约定
5. 注释
6. 格式
7. C++特性
8. 版本号命名规范

## 1. 头文件
### 1.1 头文件保护
头文件保护用于防止头文件被多重包含。为保证唯一性，头文件保护约定为：<PROJECT>_<PATH>_<FILE>_H_

```
#ifndef SRC_FORWARDX_CORE_SRC_GFSM_EVENT_HPP_  
#define SRC_FORWARDX_CORE_SRC_GFSM_EVENT_HPP_  
...  
#endif
```
部分IDE可以自动生成头文件保护
### 1.2 前置声明
前置声明仅声明类、函数和模板，不包括其定义。尽可能地避免使用前置声明，使用 #include 包含需要的头文件即可。这样做可能导致编译时间较长（与包含的头文件相关的文件均会重新编译），但不会跳过必要的编译过程。
### 1.3 内联函数
当函数被声明为内联函数之后, 编译器会将其内联展开, 而不是按通常的函数调用机制进行调用  

只要内联的函数体较小, 内联该函数可以令目标代码更加高效. 对于存取函数以及其它函数体比较短, 性能关键的函数, 鼓励使用内联。  

一个较为合理的经验准则是, 不要内联超过 10 行的函数.  
（注意：函数体大小指函数展开后的大小，不是代码行数。包含循环、递归结构的函数不应该被定义为内联函数。）
### 1.4 头文件包含顺序
使用标准的头文件包含顺序可增强可读性, 避免隐藏依赖。项目内头文件应按照项目源代码目录树结构排列, 避免使用 UNIX 特殊的快捷目录 . (当前目录) 或 .. (上级目录).   
以Foo类为例（头文件为foo.h，源文件为foo.cpp），头文件包含次序（使用空行区分头文件类型）：
> 1. 类定义头文件 (foo.h)  

> 2. C系统文件  
> 3. C++系统文件  

> 4. 其他库的头文件  

> 5. 本项目内的头文件  

## 2. 作用域
### 2.1 命名空间
命名空间将全局作用域细分为独立的, 具名的作用域, 可有效防止全局作用域的命名冲突.例如两个不同的协作项目都在全局作用域中定义了类Foo，这样在编译时会造成冲突。原则上，只引入命名空间中必要的符号，尽量将引入的符号限制在最小范围内（例如，如果仅在某个函数中使用到其他命名空间中的符号，则只在这个函数内部引入需要的符号）。

对命名空间给出如下约定：  
> 1. 必须为项目定义专用命名空间
> 2. 禁止在std命名空间中声明任何东西
> 3. 禁止在全局作用域内使用using namespace方式引入整个命名空间  

在实际使用中推荐采用namespace::symbol的方式，例如在命名空间project1中定义了类Foo,在命名空间project2中使用Foo类：  

```
namespace project2 {
...
project1::Foo* f_ptr = new project1::Foo();
...
}
```
  
### 2.2 局部变量
将局部变量尽可能放置在最小作用域内，并在变量声明时进行初始化。简言之，离第一次使用越近越好。  

但是，位于循环体内的变量应该在循环体外部声明并定义，避免频繁调用构造析构函数、动态分配内存导致的性能低下。   

```
for (int i = 0; i < 1000; ++i) {
	Foo f;  // 构造和析构函数分别调用1000次
	f.DoSth(i);  
	
	Foo* f = new F();  // 进行1000次动态内存分配和释放
	f->DoSth(i);
	delete f;
}
```
### 2.3 静态和全局变量
静态生存周期的对象，即包括了全局变量，静态变量，静态类成员变量和函数静态变量，都必须是原生数据类型 (POD : Plain Old Data): 即 int, char 和 float, 以及 POD 类型的指针、数组和结构体。  

因为多编译单元中的静态变量执行时的构造和析构顺序是未明确的，这将导致代码的不可移植。  

如果必须使用自定义类型全局变量，请采用单例模式。   

## 3. 类
### 3.1 构造函数
构造函数用于构建对象，对构造函数给出如下约定：
> 1. 禁止在构造函数内调用自身虚函数    
> 2. 必须对类的所有成员变量进行列表初始化  
> 3. 避免构造函数出现错误，导致初始化失败的对象，并进一步导致不确定行为  
> 4. 如果对象需要进行有意义的初始化，考虑使用明确的初始化方法（如提供Init()方法）  
> 5. 单参数构造函数应声明为explict，避免隐式类型转化

### 3.2 析构函数
析构函数用于对象清理，释放资源，必须将类的析构函数声明为虚函数，使得对象的基类成分能够释放。
### 3.3 拷贝/移动构造函数和拷贝/移动赋值运算符
除非必要，否则应禁止对象的拷贝/移动构造操作和拷贝/移动赋值运算符。在绝大多数情况下，拷贝/移动操作都是不必要的。  

如果没有定义拷贝/移动构造函数和拷贝/移动赋值运算符，编译器会生成默认的拷贝/移动构造函数和拷贝/移动赋值运算符，并不一定保证与期望操作一致（例如默认实现不会提供深度拷贝），导致令人困惑并难以诊断出的错误。  

如果定义了拷贝/移动操作, 则要保证这些操作的默认实现是正确的。  

有多种方式可以实现禁用拷贝/移动，如下提供一种方式，也可以继承boost提供的noncopyable类。
  
```
// MyClass is neither copyable nor movable.
MyClass(const MyClass&) = delete;
MyClass& operator=(const MyClass&) = delete;
```
### 3.4 继承
继承塑模的是"is a"概念，只有确保满足该条件时才使用继承，否则应该使用组合("has a")。禁止滥用继承。

### 3.5 多重继承
真正需要用到多重继承的情况少之又少. 有时多重实现继承看上去是不错的解决方案, 但这时你通常也可以找到一个更明确, 更清晰的不同解决方案。  

只有当所有父类除第一个外都是 纯接口类 时, 才允许使用多重继承. 为确保它们是纯接口, 这些类必须以 Interface 为后缀.  

### 3.6 接口
满足特定条件的类, 这些类应以 Interface 为后缀： 
* 只有纯虚函数 (“=0”) 和静态函数 (除了下文提到的析构函数).
* 没有非静态数据成员.
* 没有定义任何构造函数. 如果有, 也不能带有参数, 并且必须为 protected.
* 如果它是一个子类, 也只能从满足上述条件并以 Interface 为后缀的类继承.
 
### 3.7 存取控制
除非必要，类的所有数据成员均应声明为private或protected类型，并提供存取函数，以提高类的封装性。

### 3.8 声明顺序
类定义一般应以 public: 开始, 后跟 protected:, 最后是 private:。 省略空部分。
  
在各个部分中, 建议将类似的声明放在一起, 并且建议以如下的顺序: 类型 (包括 typedef, using 和嵌套的结构体与类), 常量, 工厂函数, 构造函数, 赋值运算符, 析构函数, 其它函数, 数据成员.

## 4. 命名约定
### 4.1 通用命名规则
* 函数命名, 变量命名, 文件命名要有描述性 
* 少用缩写，采用缩写时应使用常见公认缩写  

### 4.2 文件命名
* 文件名全部小写，单词之间以下划线（_）分割
* 文件名应能简要表明文件内容，文件名应与类名一致
* 头文件以.h结尾，源文件以.cpp结尾
例如类FooBar类，其头文件应命名为foo\_bar.h，源文件应命名为foo\_bar.cpp
  
### 4.3 类型命名
所有类型命名 —— 类, 结构体, 类型定义 (typedef), 枚举, 类型模板参数 —— 均使用相同约定, 即以大写字母开始, 每个单词首字母均大写, 不包含下划线. 例如:
  
```
class UrlTable { ...
class UrlTableTester { ...
struct UrlTableProperties { ...

// 类型定义
typedef hash_map<UrlTableProperties *, string> PropertiesMap;

// using 别名
using PropertiesMap = hash_map<UrlTableProperties *, string>;

// 枚举
enum UrlTableErrors { ...
```
### 4.4 变量命名
**普通变量**  
普通变量名一律小写，单词之间以下划线（\_）连接，例如:  

```
std::string table_name;
```
**类成员变量**
不管是静态的还是非静态，类成员变量名一律小写，以m开头，单词之间以下划线(\_)连接，例如：  

```
...
std::string m\_table\_name;
...
```
**结构体成员变量**
与普通变量命名一致，不区分静态成员变量和非静态成员变量

### 4.5 常量命名
常量命名以 “k” 开头, 大小写混合. 例如:  

 ```
 const int kDaysInAWeek = 7;
 ```
### 4.6 函数命名
函数命名使用驼峰命名法，所有单词首字母大写，例如：  

```
AddTableEntry()
DeleteUrl()
OpenFileOrDie()
```
存取函数命名与变量一致. 一般来说它们的名称与实际的成员变量对应, 但并不强制要求. 例如:
  
```
 int count();
 void set_count(int count);
```
### 4.7 命名空间命名
命名空间以小写字母命名，单词之间以下划线(\_)连接。通常一个项目仅使用一个命名空间，以项目名称命名。若项目规模较大，为避免名字冲突，可以使用子命名空间。
### 4.8 枚举命名
枚举命名全部字母大写，单词之间以下划线(\_)分割。例如：

```
enum class AlternateUrlTableErrors {
    OK = 0,
    OUT_OF_MEMORY = 1,
    MALFORMED_INPUT = 2,
};
```
### 4.9 宏命名
宏命名与枚举命名一致。  
**除非必要，禁止使用宏定义。**

## 5. 注释
对保证代码可读性至关重要，但同时应记住，最好的代码应当本身就是文档，所以注释应简明扼要，切忌啰嗦。

**注释通常是较为简短的，如果你需要大量的注释来说明意图，那说明需要考虑重新设计一下了**
### 5.1 注释风格
三行及以内的注释，建议使用行注释："//"  
三行以上的注释，建议使用块注释: "/\*...\*/"
### 5.2 文件注释
文件注释包括版权公告、作者信息和文件概要。请采用如下的文件注释，并更改作者信息和文件概要：

```
/*************************************************************************
*
* RENAISSANCE ROBOT LLC CONFIDENTIAL
* __________________
*
*  [2018] RENAISSANCE ROBOT LLC
*  All Rights Reserved.
*
* NOTICE:  All information contained herein is, and remains the property of
* Renaissance Robot LLC and its suppliers, if any. The intellectual and
* technical concepts contained herein are proprietary to Renaissance Robot LLC
* and its suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law.
*
* Dissemination of this information or reproduction of this material is strictly
* forbidden unless prior written permission is obtained from Renaissance Robot LLC.
*
* Author: author
*  Email: autor@example.com
*   Date: yyyy.mm.dd
*  Brief: A brief introduction to this file.
* 		   
*/

```
其中，“[2018] RENAISSANCE ROBOT LLC”中的年份“[2018]”为本文件最后一次修改日期。  
Brief部分如果超过一行，请换行后保持文本部分左对齐。
### 5.3 类注释
每个类都要有一个类注释。类注释位于类的开头部分，简要说明类的功能以及类特性。  

```
/* 
 * Iterates over the contents of a GargantuanTable.
 */
class GargantuanTableIterator {
  ...
};
```
### 5.4 函数注释
函数声明处的注释描述函数功能; 定义处的注释描述函数实现。不强制要求所有的函数都需要加注释，应保证添加的注释有意义，否则直接省略。
   
**函数声明注释**  
仅描述函数功能，不描述函数如果工作。例如：  

```
/*
 *  brief: 计算两个数的和 
 *  param: a 操作数
 *  param: b 操作数
 * return: 求和结果
 */
int Add(int a, int b);
```
(这是个不好的例子，通过函数名可以直接理解函数功能，可以省略注释)
  
**函数实现注释**  
函数注释用以说明函数是如何完成功能的。如果函数的实现过程中用到了很巧妙的方式, 那么在函数定义处应当加上解释性的注释. 例如, 你所使用的编程技巧, 实现的大致步骤, 或解释如此实现的理由. 举个例子, 你可以说明为什么函数的前半部分要加锁而后半部分不需要。

### 5.5 变量注释
通常变量名本身足以很好说明变量用途。某些情况下, 也需要额外的注释说明。变量注释位于变量声明上一行：

```
// Used to bounds-check table accesses. -1 means
// that we don't yet know how many entries the table has.
int num_total_entries_;
```
### 5.6 实现注释
对于代码中巧妙的, 晦涩的, 有趣的, 重要的地方加以注释。
### 5.7 TODO注释
对那些临时的, 短期的解决方案, 或已经够好但仍不完美的代码使用 TODO 注释。建议在TODO注释后添加名字，便于查找。  

```
// TODO(author): change pass by value to reference
```
### 5.8 修改注释
在协作开发过程中，不可避免的要修改代码，无论是自己的还是他人的。必须对所有的代码修改（曾删改）进行注释。修改注释应包括修改日期(yyyy.mm.dd)、姓名、ID（例如JIRA系统的Issue ID，可省略）、简要说明。  

修改注释表明修改人对修改的代码负责。
若仅修改一行，应在修改行行尾：

```
auto iter = std::find(v.begin(), v.end(), element);
if (iter != v.end() && !IsAlreadyProcessed(element)) {  // 2018.11.22 Sam Bug_101: 应判断element是否已处理
  Process(element);  
}
```

若修改多行，应以大括号指明修改范围：

```
// 2018.11.22 SteveJobs Bug_101: 对每个元素进行处理... {
for (auto iter = v.begin(); iter != v.end(); ++iter) {
	Porcess(*iter);
	Func(*iter);
}
// } 2018.11.22 SteveJobs Bug_101
```
## 6. 格式
### 6.1 行长度
每一行代码字符数不超过80。
### 6.2 空格还是制表符
使用空格缩进，每次缩进4个空格。可以在IDE中将制表符转为空格。
### 6.3 函数声明与定义
返回类型和函数名在同一行, 参数也尽量放在同一行, 如果放不下就对形参分行。  

```
ReturnType ClassName::FunctionName(Type par_name1, Type par_name2) {  
    DoSomething();
    ...
}
```

如果同一行文本太多, 放不下所有参数:  

```
// 换行的参数应与上一行参数左对齐
ReturnType ClassName::ReallyLongFunctionName(Type par_name1, Type par_name2,
                                             Type par_name3) {
    DoSomething();
    ...
}
```
若第一个参数也放不下：  

```
ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
        Type par_name1,  // 8 space indent
        Type par_name2,
        Type par_name3) {
    DoSomething();
  ...
}
```
注意以下几点：

* 使用好的参数名
* 如果返回类型和函数名在一行放不下, 分行
* 如果返回类型与函数声明或定义分行了, 不要缩进
* 左圆括号总是和函数名在同一行
* 函数名和左圆括号间永远没有空格
* 圆括号与参数间没有空格
* 左大括号总在最后一个参数同一行的末尾处, 不另起新行
* 右大括号总是单独位于函数最后一行, 或者与左大括号同一行
* 右圆括号和左大括号间总是有一个空格
* 所有形参应尽可能对齐  
* 逗号后必定有空格，左大括号前必定有空格

### 6.4 lambda表达式
Lambda 表达式对形参和函数体的格式化和其他函数一致; 捕获列表同理, 表项用逗号隔开。  

若用引用捕获, 在变量名和 & 之间不留空格。  

```
int x = 0;
auto add_to_x = [&x](int n) { x += n; };
```


```
...
std::set<int> blacklist = {7, 8, 9};
std::vector<int> digits = {3, 9, 1, 8, 4, 7, 1};
digits.erase(std::remove_if(digits.begin(), digits.end(), [&blacklist](int i) {
    return blacklist.find(i) != blacklist.end();
}),
digits.end());
...
```
注意以下几点：
* 捕获列表、形参列表和左大括号应位于同一行
* 左大括号下一行写函数语句，缩进4个空格
* 右大括号所在行最好不包括语句或参数  

### 6.5 函数调用
要么一行写完函数调用, 要么在圆括号里对参数分行, 要么参数另起一行且缩进4格. 如果没有其它顾虑的话, 尽可能精简行数, 比如把多个参数适当地放在同一行里。

```
bool retval = DoSomething(argument1, argument2, argument3);
bool retval = DoSomething(averyveryveryverylongargument1,
                          argument2, argument3);
                          if (...) {
...
...
if (...) {
    DoSomething(
        argument1, argument2,  // 4 空格缩进
        argument3, argument4);
  }
```
### 6.6 列表初始化格式
平时怎么格式化函数调用, 就怎么格式化 列表初始化。
### 6.7 条件语句
不在圆括号内使用空格，关键字 if 和 else 另起一行。大括号禁止省略。

```
if (condition) {  // 圆括号里没有空格.
    ...  // 4 空格缩进.
} else if (...) {  // else 与 if 的右括号同一行.
    ...
} else {
    ...
}
```
### 6.8 开关选择语句

case的大括号可用可不用，建议不使用。  
switch与左圆括号之间空1格，右圆括号与左大括号之间空一格。

```
switch (var) {   // 注意圆括号旁边的空格
    case 0:      // 4 空格缩进
        ...      // 8 空格缩进
        break;
    case 1:
        ...
        break;
    default:
        ...		
}
```
### 6.9 循环语句
循环语句的大括号禁止省略。

```
for (int i = 0; i < kSomeNumber; ++i) {  // 注意圆括号旁边的空格
    printf("I take it back\n");
}

while (condition) {  // 注意圆括号旁边的空格
    ...
}
```
### 6.10 指针和引用
在声明指针变量或参数时, 星号与类型或变量名紧挨都可以，但是不能两边都有空格

```
// 好, 空格前置
char *c;
const string &str;

// 好, 空格后置
char* c;
const string& str;

// 差 - & 两边都有空格
const string & str;  
```
### 6.11 布尔表达式
如果一个布尔表达式超过 标准行宽, 断行方式要统一， 逻辑与 (&&) 操作符总位于行尾，表达式左对齐：

```
if (this_one_thing > this_other_thing &&
    a_third_thing == a_fourth_thing &&
    yet_another && last_one) {
  ...
}
```

### 6.12 预处理指令：
预处理指令不要缩进, 从行首开始，#后不要跟空格。  

```
if (lopsided_score) {
#if DISASTER_PENDING      // 正确 - 从行首开始
    DropEverything();
#if NOTIFY              
    NotifyClient();
#endif
#endif
    BackToNormal();
}
```
### 6.13 类格式 
访问控制块的声明依次序是 public:, protected:, private:，不缩进。

```
class MyClass : public OtherClass {
public:         // 注意不缩进
    MyClass();  // 4空格缩进
    explicit MyClass(int var);
    ~MyClass() {}

    void SomeFunction();
    void SomeFunctionThatDoesNothing() {}

    void set_some_var(int var) { some_var_ = var; }
    int some_var() const { return some_var_; }

private:
    bool SomeInternalFunction();

    int some_var_;
    int some_other_var_;
};

```
注意一下几点：
* 所有基类名尽量与子类放在同一行
* 除public外，private和protected关键字前空一行、后不空行
* 不同类型的成员函数之间建议添加空行，例如存取函数应集中声明，并与其他类型函数间空一行

### 6.14 构造函数初始化列表
建议采用列表初始化的方式初始化全部类的成员变量，不要放在构造函数中初始化，可以提供专门的初始化方法。
若所有变量都能放在同一行：

```
MyClass::MyClass(int var) : some_var_(var) {
  DoSomething();
}
```
若初始化列表需要置于多行，则应每个成员变量单独一行，并缩进4个空格：

```
MyClass::MyClass(int var) :         // 冒号与函数名在同一行
        some_var_(var),             // 8 space indent
        some_other_var_(var + 1) {  // lined up
    DoSomething();
}
```
### 6.15 命名空间格式
命名控件内容不缩进。

```
namespace foo{

void foo() {  // 正确. 命名空间内没有额外的缩进.
  ...
}

}  // namespace foo
```
### 6.16 水平留白
水平留白的使用根据在代码中的位置决定. 永远不要在行尾添加没意义的留白。  
**通用**

```
void f(bool b) {  // 左大括号前总是有空格
  ...
int i = 0;  // 分号前不加空格
// 列表初始化中大括号内的空格是可选的，如果加了空格, 那么两边都要加上
int x[] = { 0 };
int x[] = {0};

// 继承与初始化列表中的冒号前后恒有空格
class Foo : public Bar {
public:
    // 对于单行函数的实现, 在大括号内加上空格，然后是函数实现
    Foo(int b) : Bar(), baz_(b) {}  // 大括号里面是空的话, 不加空格
    void Reset() { baz_ = 0; }      // 用括号把大括号与实现分开
    ...
```
**循环和条件语句**

```
if (b) {          // 条件语句和循环语句关键字后均有空格.
} else {          // else 前后有空格.
}

while (test) {}   // 圆括号内部不紧邻空格.

switch (i) {

for (int i = 0; i < 5; ++i) {

for ( int i = 0; i < 5; ++i ) {

for (; i < 5; ++i) {  // 循环里，“;”后恒有空格、“;”前无空格
switch (i) {
    case 1:         // switch case 的冒号前无空格.
        ...
    case 2: break;  // 如果冒号后有代码, 加个空格.
```
**操作符**

```
x = 0;  // 赋值运算符前后总是有空格

// 其它二元操作符也前后恒有空格, 圆括号内部没有紧邻空格
v = w * x + y / z;
v = w * (x + z);

// 在参数和一元操作符之间不加空格
++x;
if (x && !y)
    ...
```
**模板和转换**

```
// 尖括号(< and >) 不与空格紧邻, < 前没有空格, > 和 ( 之间也没有.
vector<string> x;
y = static_cast<char*>(x);

// 在类型与指针操作符之间留空格也可以, 但要保持一致.
vector<char *> x;
```
### 6.17 垂直留白
垂直留白越少越好，如下情况下考虑垂直留白：
* 类的同一访问级别下的成员函数与成员变量之间加空行
* 类的同一访问级别下的不同种类成员函数间可以加空行
* 命名空间声明后空一行

添加空行的根本目的是增强可读性，不要添加无意义的空行。

## 7. C++特性
有关C++特性，请移步[其他 C++ 特性](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/others/)
## 8. 版本号命名规范
参考自[语义化版本](https://semver.org/lang/zh-CN/)。  

版本格式：主版本号.次版本号.修订号-先行版本号，例如：1.0.1\_beta.5。解释如下：
* 主版本号：软件有重要变更（例如大规模的重构，大规模功能扩展等）
* 次版本号：新增软件功能时递增
* 修订号：有任何修改时递增
* 先行版本号：pre-release用于内部版本迭代
  
主版本号、次版本号、修订号均为数字(^[0-9]*$)  
先行版本号可选:alpha,beta,rc  

有关先行版本号:  
**alpha**  
为研发内测版本，可经历多个版本迭代（例如分几个版本完成全部开发），研发内测通过后，软件转为beta.0版本。    
**beta**  
为提供给测试部门的版本，经历多个版本迭代。beta版本软件可能还会面临较多修改，以解决bug，完善功能。全部问题解决后，软件转为rc.0版本。  
**rc**
为预发布版本，同样会经历几个版本迭代，通常rc版本不应有大的修改，以稳定为主，即使发现重大问题，也应规划到下一个版本中。rc版本最终形成提供给用户的发布版本。  

说明：
* 项目立项时的版本号为0.0.0
* 开发阶段，系统尚不稳定，随时可能有重大修改，此时，主版本号为0，版本号为0.x.y
