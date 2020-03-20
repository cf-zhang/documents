目的：

随着仓库使用的越来越久，仓库中的子目录越来越多，仓库也变得越来越大。会导致pull和push的时间变长，共享代码就会变得很痛苦。

可以用将子目录独立成一个submodule的方式，保留分支和提交历史。
步骤：

1. Clone仓库到本地目录

    $ git clone http://gitlab.xxx/xxx.git

仓库结构：

    myrepo

     --test1

    --src

    --...

我想要把test1这个子目录submodule化

2. 选择要保留的分支

通常刚clone下来的仓库本地只会有一个branch：master，如果我们希望在马上要做的子模块中保存其他的分支，那就首先把它们创建出来：

    $ git checkout release

    $ git checkout fix

3. 删除remote信息

    $ git remote rm origin

4. 转化成submodule

这是最重要的一步

    $ git filter-branch --tag-name-filter cat --prune-empty --subdirectory-filter test1 -- --all

该命令过滤所有历史提交，保留对test1子目录有影响的提交，并且把子目录设为该仓库的根目录。下面解释下各参数意思：

--tag-name-filter cat 该参数控制我们要如何保存旧的tag，参数值为bash命令，cat表示原样输出。所以，如果你不关心tag，就不需要这个参数了；

--prune-empty 删除空的（对子目录没有影响的）的提交

--subdirectory-filter test1指定子模块路径

-- --all 该参数必须跟在--后面，表示对所有分支做操作，即对上一步创建的所有本地分支做操作。所以，如果你只想保存当前分支，就不需要这个参数了

该命令执行完毕后，查看当前目录结构就会发现里面已经是子目录的内容了。git log查看提交历史已经正常保存了。
清理：

上面的步骤完成后，当前仓库里还保存了很多不需要的object, 需要做下清理，才能减少当前仓库的体积。

    $ git for-each-ref --format="%(refname)" refs/original/ | xargs -n 1 git update-ref -d

    $ git reflog expire --expire=now --all

    $ git gc --aggressive --prune=now

提交：

1. 首先要在gitlab上创建test1的仓库

2. 对应remote

    $ git remote add origin http://xxxx/test1.git

3. 提交

    $ git push -u origin --all

    $ git push -u origin --tags

添加submodule

1. 再clone一份原来的仓库

2. 删除子目录test1

3. 把test1以submodule的方式添加

    $ git submodule add -b master http://gitlabxxxx/test1.git test1

4. 提交修改

作者：shyingsheng
链接：https://www.jianshu.com/p/aae191c821e6
来源：简书
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。

当一个git项目包含子模块（submodule）时，直接克隆下来的子模块目录里面是空的。
有两种方法解决：
方法一：
    1、初始化本地子模块配置文件
    git submodule init
    2、更新项目，抓取子模块内容。
    git submodule update

方法二：
    更简单的方法，执行git clone的时候加上 `--recursive` 参数。它会自动初始化并更新每一个子模块。例如：
    ```git clone --recursive https://github.com/xxxx/xxxx.git```
    




