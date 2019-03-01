cmockery项目中依赖系统的输出系统实现了自己的打印日志的功能，基本的功能接口如下：

```
// Standard output and error print methods.
void print_message(const char* const format, ...);
void print_error(const char* const format, ...);
void vprint_message(const char* const format, va_list args);
void vprint_error(const char* const format, va_list args);
```

其实现代码也很简单，基于vsnprintf系统调用来进行简单的一层封装。

```
// Standard output and error print methods.
void vprint_message(const char* const format, va_list args) 
{
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    puts(buffer);
}
​
​
​
​
void vprint_error(const char* const format, va_list args) 
{
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    fputs(buffer, stderr);
}
​
​
​
​
void print_message(const char* const format, ...) 
{
    va_list args;
    va_start(args, format);
    vprint_message(format, args);
    va_end(args);
}
​
​
void print_error(const char* const format, ...)
{
    va_list args;
    va_start(args, format);
    vprint_error(format, args);
    va_end(args);
}
```

    其实对于上诉的几个函数的实现，基本上没有什么可说的，全部当成printf来用就可以了，按照上诉的实现逻辑我们所要使用的只需要是print_message和print_error两个接口就可以了，实际上整个cmockery项目也是只用到了这两个对外提供的接口。

    而对于可变参数的理解就是函数的压栈过程的理解，在函数执行的时候需要将函数的参数，函数的返回地址以及函数的局部变量在栈上进行空间的分配的赋值，具体的顺序我们可以不去深究。而在函数的参数进行空间分配和赋值的时候一般是按照反向的顺序进行压栈，即先将最后一个参数存到栈中，然后将倒数第二个参数再压到栈中，直到将所有的参数全部入栈（其实正向压栈也无所谓的，只不过实现过程不一致罢了）。所以当我们理解了这一特性之后，可以根据一个已知的参数位置，和相邻的参数的类型，按部就班的找到与其相邻的参数的地址，从而得到这个参数的值。上面出现的va_*一系列的功能其实是一组宏，用来提供按部就班的寻找指定参数在栈中位置的功能。这就要求我们的可变参数设计的时候，必须能够提供每一个可变参的类型顺序（format中%d %c 之类的转意符），以及一个固定位置的参数（format）。

注意：可变参...必须位于参数列表的最后一个位置。

```
typedef char * va_list; 
​
#define _INTSIZEOF(n)   ((sizeof(n)+sizeof(int)-1)&~(sizeof(int) - 1) ) 
#define va_start(ap,v) ( ap = (va_list)&v + _INTSIZEOF(v) )           //第一个可选参数地址
#define va_arg(ap,t) ( *(t *)((ap += _INTSIZEOF(t)) - _INTSIZEOF(t)) ) //下一个参数地址
#define va_end(ap)    ( ap = (va_list)0 )                            // 将指针置为无效
```

    上面是这一组宏的一种实现方式，可以看到va_list其实就是一个指针，至于为什么用char*指针而不用void*其实我觉得都是可以的。没必要去深究这个指针的类型，只要能够存放我们任意的栈内存地址就可以了。

    定义_INTSIZEOF(n)主要是为了某些需要内存的对齐的系统

    va_start(ap,v)所做的工作就是将ap定位到v参数之后的第一个可变参地址的位置；

    va_arg(ap,t)所做的工作就是取当前ap所指的栈内存中类型为t的参数值，然后将ap再指向下一个可变参的地址；

    va_end(ap) 就是个鸡肋，将指针置空而已。
