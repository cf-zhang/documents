# 【Travis CI使用教程】如何让定制你的travis.yml配置文件

  travis ci为每一种语言提供了一个默认的构建环境和一个默认的阶段集合。它能够为项目job的构建活动创建虚拟机，并将项目仓库在虚拟机中克隆，安装可选的插件，并运行构建过程的各个阶段。

## 构建
  .travis.yml文件描述了构建的整个过程。在TravisCI中的每一个构建都是由一系列阶段（stage）所组成的。而这些阶段又被分成了不同的并行的job来运行。

## job的生命周期
  job的生命周期也是由多个阶段（phase）组成的。主要的阶段包括：
>+ Install - 安装所需的依赖
>+ Script - 执行构建脚本

   除此之外，还可以选择以下的命令定制自己的构建过程：
>+ Before-install - 在安装阶段之前所需要做的步骤
>+ Before-script - 在执行脚本之前所需要做的步骤
>+ After-script - 在执行脚本之后所需要做的步骤
>+ After-success - 当构建成功时（比如生成构建文档），在travis_test_result环境变量下的结果
>+ After-failure - 当构建失败时（比如上传日志文件），在travis_test_result环境变量下的结果

   除此之外，还有三种可选的部署阶段命令。

   一个完整的job生命周期所包含的步骤如下所示：
>+ 可选安装 apt addons
>+ 可选安装 cache components
>+ Before_install
>+ install
>+ Before_script
>+ Script
>+ 可选 before_cache(只有当缓存生效的时候)
>+ after_success或者after_failure
>+ 可选 before_deploy(只有当deploy被激活时)
>+ 可选deploy
>+ 可选after_deploy(只有当deploy被激活时)
>+ After_script

## 定制你的安装（install）阶段

   默认的依赖安装命令取决于项目所使用的语言。比如Java构建通常会使用maven或者gradle，这取决于仓库中的构建文件。使用Ruby的项目如果在仓库中存在Gemfile，会配置Bundler安装命令。

   你可以定制自己的脚本去安装项目依赖：
```bash
    install: ./install_dependencies.sh
```
   当使用自定义的脚本时，脚本的权限需要设置为可执行（比如使用chmod +x命令），并且需要包含一个有效的事务行比如：/usr/bin/env sh, /usr/bin/env ruby, 或者/usr/bin/env python。

   除此之外，你还可以提供多个步骤，比如可以安装ruby和node两种依赖：
```bash
    install:
      bundle install -- path vendor/bundle
      npm install
```

   如果设置了多种安装依赖，当其中一个安装失败时，整个构建过程就会立即终止并且被标记为errored。

   除此之外，你还可以使用Apt-get或者snap去安装依赖。

## 跳过安装（install）阶段

   通过向travis.yml中添加命令来跳过整个安装阶段：
```bash
    install: skip
```

## 定制你的构建（build）阶段

   默认的构建命令取决于项目所使用的语言。比如Ruby语言使用的构建命令是rake，这是大多数Ruby语言最常见的命令方式。

   你可以在配置文件travis.yml中重写默认构建：
```bash
    script: bundle exec thor build
```

   你可以使用命令规定多个构建脚本：
```bash
    script:
      - bundle exec rake build
      - bundle exec rake builddoc
```

   如果这些构建命令的其中一个返回非0的结果，travis CI构建机制会自动运行接下来的命令并且累计构建结果。

   比如上面的例子，命令bundle exec rake build命令返回的状态码为1，环境仍然会继续运行bundle exec rake builddoc命令，但是最后的构建结果仍然是失败（failure）。

   这种机制适用于一些场景。比如，你的第一步想执行单元测试，第二步想执行集成测试，如果第一步单元测试失败了，你仍然想知道集成测试是否能够成功。

   当然你也可以通过修改命令去改变这种机制：
```bash
    script:
      - bundle exec rake build && bundle exec rake builddoc
```

   通过这种写法，如果bundle exec rake build返回非0的结果，那么整个构建就会立即失败。

## 复杂的构建命令

   如果你的构建环境过于复杂以致于很难在travis.yml配置文件中进行配置，可以考虑写一个独立的shell脚本。可以将脚本放在你的项目的仓库中，使它能够轻易地被配置文件travis.yml调用。

   比如这样一个场景。你想执行更多更复杂的测试场景，但是想排除来自于pull request的构建，那么你可以建立这样一个shell文件：
```bash
    #!/bin/bash
    set -ev
    bundle exec rake:units
    if [ "${TRAVIS_PULL_REQUEST}" = "false" ]; then
      bundle exec rake test:integration
    fi

```

   如何在travis.yml中调用这个shell文件呢？
>+ 将shell文件在项目仓库中存为scripts/run_tests.sh
>+ 运行命令chmod ugo+x scripts/run_tests.sh为文件添加可执行权限
>+ 提交到仓库中
>+ 添加到travis.yml文件中（代码如下）
```bash
    script: ./scripts/run-tests.sh
```

## 中断构建

   如果job的生命周期的前四个步骤返回非0的状态吗，那么本次构建就会被中断。可以分成以下两种情况：
>+ 如果before_install, install, 和before_script三个阶段返回非0状态码，本次构建为errored并且立即终止
>+ 如果script阶段返回非0状态码，本次构建为失败failed，但是直到被标记为failed之前仍然在运行

   阶段after_success, after_failure, after_script, 以及after_deploy以及其后续的阶段都不对最后的构建结果产生影响。但是如果这些阶段的其中一个发生了超时，那么构建结果会被标识为failed。

## 部署代码

   在job的生命周期中部署是可选步骤。这个步骤被定义为使用一种持续部署工具将代码部署到Heroku, Amazon或者其他的不同的平台上。如果构建步骤中断了，那么部署阶段就不会被执行。

   配置代码如下：
```bash
    deploy:
      skip_cleanup: true

```

   你也可以在执行deploy命令之前执行before_deploy命令。如果这个阶段的返回状态是非0，那么整个构建为errored。

   如果你在部署之后需要做一些事情，也可以通过after_deploy命令来配置。

   但是after_deploy命令的返回结果是不影响整个构建的结果的。
