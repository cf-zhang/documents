所分析的run_tests.c文件位于 工程中的 cmockery/src/example/ 目录下，仅仅只有十几行代码，不过可以通过这十几行代码的分析达到管中窥豹的效果；

run_tests.c:
```
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmockery.h>
​
​
// A test case that does nothing and succeeds.
void null_test_success(void **state) {
}
​
​
int main(int argc, char* argv[]) {
    const UnitTest tests[] = {
        unit_test(null_test_success),
    };
    return run_tests(tests);
}
```

对于代码中出现的几个部件进行分解开来看，如下：

UnitTest结构体:
```
// 函数原型用于setup, test and teardown functions.
typedef void (*UnitTestFunction)(void **state);
​
​
// 单元测试函数的类型
typedef enum UnitTestFunctionType {
    UNIT_TEST_FUNCTION_TYPE_TEST = 0,
    UNIT_TEST_FUNCTION_TYPE_SETUP,
    UNIT_TEST_FUNCTION_TYPE_TEARDOWN,
} UnitTestFunctionType;
​
​
/* 
 * 存储一个单元测试函数包括其函数名和类型
 * 注意:每设置一个函数必须有一个teardown函数对应,可以指定一个NULL函数指针
 */
typedef struct UnitTest {
    const char* name;
    UnitTestFunction function;
    UnitTestFunctionType function_type;
} UnitTest;
```

可以看到这个结构体中所包含的内容只有一个函数名，一个函数指针，一个函数的类型枚举；


unit_test：是一个宏体，定义如下：
```
// 初始化一个单元测试结构体.
​
#define unit_test(f) { #f, f, UNIT_TEST_FUNCTION_TYPE_TEST }
```

所有上面main.c中的代码行：unit_test(null_test_success)，按照宏定义进行展开后表示如下：

{"null_test_success", null_test_success, UNIT_TEST_FUNCTION_TYPE_TEST},

所以正好填充了结构体数组的第一个元素的各个值；


run_tests：是一个宏体，定义如下：
```
/*
 * 运行通过UnitTest结构体数组指定的测试，下面的样例
 * 以unit_test宏来描述了这个宏的用法
 *
 * void Test0();
 * void Test1();
 *
 * int main(int argc, char* argv[]) {
 *     const UnitTest tests[] = {
 *         unit_test(Test0);
 *         unit_test(Test1);
 *     };
 *     return run_tests(tests);
 * }
 */
run_tests(tests) _run_tests(tests, sizeof(tests) / sizeof(tests)[0])
```

将代码中run_tests(tests);展开之后内容为：_run_tests(tests, 1)；

先简要介绍该函数的功能：

    在_run_tests函数中根据tests中元素的赋值情况，即类型和函数指针，进行不同的分支并调用_run_test运行该元素的函数指针所指向的函数体。并将运行的结果进行返回。

在这个过程中还会将函数执行的状态进行记录，其中包括了链表，内存分配，信号等相关的操作。后续会进行这个功能详细的分析。