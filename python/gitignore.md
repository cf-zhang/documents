# .gitignore

## 简绍
　　我们做的每个Git项目中都需要一个“.gitignore”文件，这个文件的作用就是告诉Git哪些文件不需要添加到版本管理中。比如我们项目中的npm包(node_modules)，它在我们项目中是很重要的，但是它占的内存也是很大的，所以一般我们用Git管理的时候是不需要添加npm包的。

## 常用的规则
```
/mtk/ 过滤整个文件夹
*.zip 过滤所有.zip文件
/mtk/do.c 过滤某个具体文件
```
　　以上规则意思是：被过滤掉的文件就不会出现在你的GitHub库中了，当然本地库中还有，只是push的时候不会上传。
除了以上规则，它还可以指定要将哪些文件添加到版本管理中。
```
!src/   不过滤该文件夹
!*.zip   不过滤所有.zip文件
!/mtk/do.c 不过滤该文件
```

### 配置语法：
+ 以斜杠/开头表示目录；
+ 以星号*通配多个字符；
+ 以问号?通配单个字符
+ 以方括号[]包含单个字符的匹配列表；
+ 以叹号!表示不忽略(跟踪)匹配到的文件或目录；

　　此外，git 对于 .ignore 配置文件是按行从上到下进行规则匹配的，意味着如果前面的规则匹配的范围更大，则后面的规则将不会生效；

### 示例说明

+ 规则：fd1/*

> 说明：忽略目录 fd1 下的全部内容；注意，不管是根目录下的 /fd1/ 目录，还是某个子目录 /child/fd1/ 目录，都会被忽略；

+ 规则：/fd1/*

> 说明：忽略根目录下的 /fd1/ 目录的全部内容；

+ 规则：
```
/*
!.gitignore
!/fw/bin/
!/fw/sf/
```
> 说明：忽略全部内容，但是不忽略 .gitignore 文件、根目录下的 /fw/bin/ 和 /fw/sf/ 目录；

### 创建.gitignore文件
* 常规的windows操作
>+ 根目录下创建gitignore.txt；
>+ 编辑gitignore.txt，写下你的规则，例如加上node_modules/；
>+ 打开命令行窗口，切换到根目录（可以直接在文件夹上面的地址栏输入cmd回车）；
>+ 执行命令ren gitignore.txt .gitignore。
* 用Git Bash
>+ 根目录下右键选择“Git Bash Here”进入bash命令窗口；
>+ 输入vim .gitignore或touch .gitignore命令，打开文件（没有文件会自动创建）；
>+ 按i键切换到编辑状态，输入规则，例如node_modules/，然后按Esc键退出编辑，输入:wq保存退出。

如下：
```python
# dependencies  npm包文件
/node_modules

# production  打包文件
/build

# misc 
.DS_Store

npm-debug.log*

```

## 注意事项
　　最后需要强调的一点是，如果你不慎在创建.gitignore文件之前就push了项目，那么即使你在.gitignore文件中写入新的过滤规则，这些规则也不会起作用，Git仍然会对所有文件进行版本管理。

　　简单来说，出现这种问题的原因就是Git已经开始管理这些文件了，所以你无法再通过过滤规则过滤它们。因此一定要养成在项目开始就创建.gitignore文件的习惯，否则一旦push，处理起来会非常麻烦。

