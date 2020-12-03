# Python 中的 requirement.txt

## Python 中的依赖


正如 PHP 中使用 Composer 维护依赖一样，Python 也需要维护项目相关的依赖包。通常我们会在项目的根目录下放置一个 requirement.txt 文件，用于记录所有依赖包和它的确切版本号。

requirement.txt 的内容长这样：

```python
alembic==1.0.10
appnope==0.1.0
astroid==2.2.5
attrs==19.1.0
backcall==0.1.0
bcrypt==3.1.6
bleach==3.1.0
cffi==1.12.3
Click==7.0
decorator==4.4.0
defusedxml==0.6.0
entrypoints==0.3
...
```

## 如何使用？
那么 requirement.txt 究竟如何使用呢？

当我们拿到一个项目时，首先要在项目运行环境安装 requirement.txt 所包含的依赖：

```python
pip install -r requirement.txt
```

当我们要把环境中的依赖写入 requirement.txt 中时，可以借助 freeze 命令：

```python
pip freeze >requirements.txt
```

## 环境混用怎么办？

在导出依赖到 requirement.txt 文件时会有一种尴尬的情况。

你的本地环境不仅包含项目 A 所需要的依赖，也包含着项目 B 所需要的依赖。此时我们要如何做到只把项目 A 的依赖导出呢？

pipreqs 可以通过扫描项目目录，帮助我们仅生成当前项目的依赖清单。

通过以下命令安装：
```python
pip install pipreqs
```
运行：
```python
pipreqs ./
```