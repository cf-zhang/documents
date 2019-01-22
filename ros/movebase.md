# move_base

## 概念

> move_base包为一个ation提供了实现的途径（详情参见actionlib包)。假如在地图内给定一个目标，这个操作将尝试控制一个移动基座到达这个目标。

>move_base节点将全局路径和局部路径规划程序链接在一起，以完成其全局导航任务。

>它支持连接符合nav_core包中指定的nav_core::BaseGlobalPlanner接口的任一全局路径规划器和符合nav_core::BaseLocalPlanner接口的任一局部路径规划器。

>move_base节点还维护两个costmaps，一个用于全局路径规划器，一个用于局部路径规划器（参见costmap_2d软件包），用于完成导航任务。

>move_base为使用导航功能包集将机器人移动到所需的位置提供了一个总入口，将外部传感器数据，tf关系，定位，地图，costmap，planner，recovery连贯起来

## 逻辑框架图

![导航栈逻辑框架图](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=overview_tf_small.png)

>说明：

>> move_base节点的高级视图及其与其他组件的交互如上所示。蓝色部分基于机器人平台而变化，所有平台都能提供可选择的灰色部分和不可少的白色部分。


## move_base节点进行流程

![move_base节点进行流程](http://wiki.ros.org/move_base?action=AttachFile&do=get&target=recovery_behaviors.png)

### 机器人详细行为流程

>在机器人尝试到达在使用者允许的误差范围内目标位置。

>在没有动态障碍的情况下，move_base节点最终运行结果要么是接近了规划的目标，要么是给用户返回失败信号。

>当机器人认为它是卡住了，move_base节点可以选择执行修复操作。默认情况下，move_base节点采用如下所述行动来清理出的周围空间：

>>用户指定区域以外的障碍物将从机器人的地图中清除。如果失败，机器人会更激进的将其可以就地旋转的矩形框外面的障碍物全部移除。

>>紧接着下一个就地旋转操作会被继续执行。如果所有这些操作都失效，机器人会认为目标设定不恰当并通知用户放弃操作.

>>这些修复操作可以通过recovery_behaviors参数来配置，也可以recovery_behavior_enabled参数disable。

## move base action

定义在move_base_msg包里面的action协议。用于指定导航目标点位置

## MoveBase类资源结构

 ![MoveBase资源](https://raw.githubusercontent.com/cf-zhang/documents/master/ros/image/resource.png)

---

# Locomotor

locomotor是一个可扩展的路径规划协调引擎，它取代了move-base。其目标是提供一种机制来控制当全球和本地规划者成功和失败时会发生什么。它利用ROS回调队列来协调多个线程。

## Motivation

ros-navigation栈中的move_base所存在的问题
- 使用了太多的线程
>在ROSCon 2017 (Slides) Ingo提出，从传感器数据发现新障碍到生成的计划实际发生变化之间存在不可接受的时间间隔。关键原因是move_base的所有部分都在各自独立的线程中运行。 

>-传感器数据从topic的标准回调队列中进入costmap进行处理

>-每个costmap在他们各自的线程中进行更新

>-在move_base节点中有一个单独的线程用于处理全局路径规划

>-局部路径规划是在actionlib的回调中生成的

>以上列举的问题除了导致时间上的延迟之外，我们在调试的时候也不方便


- 当全局或本地规划失败时我们什么也改变不了（不灵活）

>move_base内含有一个让人头疼的状态机逻辑.并且还不太方便对其做改变.下面的几种情况都会导致状态机的跳转

>-接收到一个新目标

>-成功规划出了一个新的全局路径

>-无法规划出全局路径

>-无法规划出局部路径

>-所有的恢复行为执行完成

>如果全局规划或者局部规划中某一个失败了，则会触发第一个恢复行为。当恢复行为完成时，它将尝试计算新的全局计划，然后计算新的本地计划。如果相同的故障再次发生，将执行恢复行为列表中的下一个恢复行为。

>恢复行为机制提供了一个通用的接口runBehavior(),但是并没有区分恢复行为是原子操作（清除costmap）还是复杂操作(可能需要任意时间的局部路径规划).

>同一个恢复行为列表应对所有的异常情况

>一般来说，恢复行为的概念没有太大的价值。它们是导航的一种特殊情况，可能有用，但实际上很难理解和实现。 

>根据Ingo的言论，我们应该要清除的知道costmap什么时候被清除完成

- 对于执行过程中发生了什么事情，我们很难知道

>有非常多的理由我应该知道move_base到底发生了什么

>-为什么全局规划失败了

>-为什么局部规划失败了

>-此次导航任务还需要执行多久

>-当前正在执行的恢复行为是哪一个

>-机器人已经重新规划过多少次了

>-机器人还可以动吗

>上面列举的一些信息由于nav_核心接口的限制而变得模糊,一些信息被move_base代码本身的结构所模糊。
 
## Primary Design

Locomor的主要思想是隔离每个action（更新成本图，制定计划），并允许用户控制当每个事件成功或失败时会发生什么。通过使用由locomotor类生成的回调，用户可以实现自己的状态机并自定义机器人的行为。

可定制性还包括控制每个操作在哪个线程中运行的能力。此包提供Executor类，该类创建新的ROS:：CallbackQueue（即线程）。最简单的版本可以用单个执行器在单个线程中使用全局CallbackQueue来运行，但可以使用任意数字。CostMap/Planner操作在特定的执行器中运行。  


### navi_core2 component

首先,机器人的目标是通过setGoal函数进行设置的.为了导航到该目标点我们需要四个主要部件，全局路径规划期，局部路径规划期，全局costmap，局部costmap

- 全局和局部规划器提供了update接口

- 全局规划器提供了makePlan接口

- 局部规划器提供了computeVelocityCommands

将以上的任意一个添加到回调函数队列中，调用request，然后实际的工作会在这个回调函数队列中的一个函数里执行完成

Component        | Request                        | Actual Execution
---------------- | ------------------------------ | -----------------
Global `Costmap` | `requestGlobalCostmapUpdate()` | `doCostmapUpdate()`
Local `Costmap`  | `requestLocalCostmapUpdate()`  | `doCostmapUpdate()`
`GlobalPlanner`  | `requestGlobalPlan()`          | `makeGlobalPlan()`
`LocalPlanner`   | `requestLocalPlan()`           | `makeLocalPlan()`

以上函数的参数会指定：

* 哪一个执行器来做实际的工作

* 当成功的时候调用哪个回调函数

* 当失败的时候调用哪个回调函数

* 在哪一个执行器的回调队列中来执行即将要执行的回调函数

`requestLocalPlan`函数还会提供一个特殊的回调函数用于指定导航任务结束的时候应该做什么，如果出现异常情况，无法抵达目标点，那么就可以调用`requestNavigationFailure`来触发一个独立的回调来处理

### Plugin Management

costmap是简单同过pluginlib来加载的，并且假设只有一个全局代价地图和一个局部代价地图

路径规划器是用Plugin Mux工具来进行加载的，这样方便进行规划器切换，不同的规划器包括全局规划器和局部规划器都被加载到不同的命名空间中，在同一时间只有一个规划器是被激活的。

这样可以对不同的目标点设定不同的规划器，如果收到停靠目标，可以将本地规划器设置为停靠本地规划器，然后尝试停靠。

### Publishing

`Locomotor` 有两个Publisher对象，用于发送规划出的全局路径和速度命令，并且允许不同的数据类型

* 全局路径Publisher可以发送nav_msgs::Path 或者 nav_2d_msgs::Path2D，或者None

* 速度命令Publisher可以发送geometry_msgs::Twist(默认) 或者nav_2d_msgs::Twist2DStamped,或者nav_2d_msgs::Twist2D 或者None

另外，全局路径可以指定一个非负数`global_plan_epsilon`(默认为0.1)在发送之前进行压缩.这个值越大，压缩比例越高

## Example State Machines

### One Thread To Rule Them All

Locomotor的最简单的版本是使用一个单线程，执行流程如下（假设一切都按计划执行）

* 1.当收到一个新的目标时，更新全局costmap

* 2.当更新完成，创建一个新的全局规划（makePlan）

* 3.当规划完成之后，将这个路径传给局部路径规划器（setPlan）
 
* 4.更新局部costmap

* 5.局部costmap更新完成之后，创建一个新的局部规划（computeVelocityCommand）

* 6.重复4和5，直到目标位置抵达

![single thread flow diagram](https://raw.githubusercontent.com/locusrobotics/robot_navigation/master/locomotor/doc/figures/single_thread.png)

一个主线程将处理所有事件并请求其他事件。 

在这个超级简单的版本中，我们将非常保守的处理异常

* 如果全局costmap失败，或者全局规划失败，整个导航任务失败

* 如果局部costmap失败， 则发送0速度，然后重新尝试

* 如果局部规划失败，那么发送0速度，然后触发下一次全局路径规划

### It Takes Two（Threads）

以上简单版本无法在不阻塞本地规划器的情况下连续创建全局规划。所以也可以制作一个具有类似逻辑的双Executor版本。 

![two thread flow diagram](https://raw.githubusercontent.com/locusrobotics/robot_navigation/master/locomotor/doc/figures/two_thread.png)

主要的不同点在于使用了两个回调函数队列，一个用于局部costmap和局部规划器，另一个用于全局costmap和全局规划器。两个线程之间通过`onNewGlobalPlan`联系起来

实现时创建两个定时器，一个用于定时执行全局costmap更新，一个用于定时触发局部costmap更新

### Other Configurations

可以设置4个执行器, 在独立的线程中控制costmap更新和规划起工作

## Error Handling

### Error Handling

先看一下异常处理的结构

`using PlannerExceptionCallback = std::function<void (nav_core2::PlannerException, const ros::Duration&)>;`

nav_core2/exceptions.h中有几个基本plannerexception类的标准扩展。调用函数时，您应该能够根据已知类型检查异常的类并做出相应的反应。例如，
```$xslt
void SomeStateMachine::onGlobalPlanningException(nav_core2::PlannerException e,
                                                 const ros::Duration& planning_time)
{
  try
  {
    throw e;
  }
  catch (nav_core2::OccupiedGoalException e)
  {
    // modify the goal and try again
  }
  catch (nav_core2::PlannerTFException e)
  {
    // wait 10 seconds, try again
  }
  catch (nav_core2::PlannerException e)  // catch all
  {
    // trigger plan failure
  }
}

```



重要的是，各个规划器可以实现他们自己对PlannerException的扩展，并让他们的StateMachine版本按他们喜欢的方式处理它。

 ```$xslt
void LocomotorBrown::onGlobalPlanningException(nav_core2::PlannerException e,
                                               const ros::Duration& planning_time)
{
  try
  {
    throw e;
  }
  catch (LocomotorBrown::TimeTravelException e)
  {
    // recalibrate flux capacitor
  }
  ...
}

```

### Recovery Behaviors

基本上有两种恢复行为。

* 简单的近乎瞬间的动作，比如清除成本图 
 
* 试图把事情弄清楚的长时间行动（例如原地旋转一点）

前者可以在错误事件处理中处理。后者可以表示为切换到一个新的计划器进行一定数量的导航。 

![error handling](https://raw.githubusercontent.com/locusrobotics/robot_navigation/master/locomotor/doc/figures/error_handling.png)

locomove_base包是使用标准nav_core::recoverybehavior配置的locomotor的一种实现，该配置用于move_base用来错误处理

## Build on Top

### Building on the Events

#### Actioin Structure

虽然locomotor是move_base的替代品，但它不使用相同的动作定义。现有的move_base_msgs/move base.action中几乎没有能用的信息。

```$xslt
# move_base_msgs/MoveBase.action
geometry_msgs/PoseStamped target_pose
---
---
geometry_msgs/PoseStamped base_position

``` 

提供一个目标姿势，你得到的唯一反馈是机器人的当前位置。当操作完成后，您会收到一个通知，所以这很”GOOD“。

locomotor_msgs定义提供了一个更丰富的action定义，还可以将自己的动作定义与locomotor的定义结合使用。

NavigateToPose.action
```$xslt
nav_2d_msgs/Pose2DStamped goal
---
ResultCode result_code
---
NavigationState state
float32 percent_complete
float32 distance_traveled
float32 estimated_distance_remaining

``` 

NavigationState.msg

```$xslt
nav_2d_msgs/Pose2DStamped global_pose
nav_2d_msgs/Pose2DStamped local_pose
nav_2d_msgs/Pose2DStamped goal
nav_2d_msgs/Twist2DStamped current_velocity
nav_2d_msgs/Twist2DStamped command_velocity
nav_2d_msgs/Path2D global_plan

```

ResultCode.msg

```$xslt
# This message contains three separate pieces.
#    A) A code indicating which component(s) the error originates from (bitmask style)
#    B) A code corresponding with the result_code defined in nav_core2/s.h
#    C) A freeform string message

# The enumerations below are not necessarily the exclusive values for the codes.
# Others may implement additional values beyond the ones shown, using custom state machines.

########### Component Values ###############################################
int32 GLOBAL_COSTMAP = 1
int32  LOCAL_COSTMAP = 2
int32 GLOBAL_PLANNER = 4
int32  LOCAL_PLANNER = 8

########### Result Codes ###################################################
int32 GENERIC_COSTMAP=0
int32 COSTMAP_SAFETY=1
int32 COSTMAP_DATA_LAG=2
int32 GENERIC_PLANNER=3
int32 GENERIC_GLOBAL_PLANNER=4
int32 INVALID_START=5
int32 START_BOUNDS=6
int32 OCCUPIED_START=7
int32 INVALID_GOAL=8
int32 GOAL_BOUNDS=9
int32 OCCUPIED_GOAL=10
int32 NO_GLOBAL_PATH=11
int32 GLOBAL_PLANNER_TIMEOUT=12
int32 GENERIC_LOCAL_PLANNER=13
int32 ILLEGAL_TRAJECTORY=14
int32 NO_LEGAL_TRAJECTORIES=15
int32 PLANNER_TF=16

########### Actual Data ####################################################
int32 component
int32 result_code
string message

```

#### Action Server

Locomotor包还提供了LocomotorActionServer类,让actionlib与状态机集成在一起，它生成合适的反馈和结果，并在合适的时机进行发送。

![action server](https://raw.githubusercontent.com/locusrobotics/robot_navigation/master/locomotor/doc/figures/action_server.png)

以上描述的是与单线程版本的逻辑一致，但是说明了整体逻辑与左侧的actionserver进行交互的流程

#### ROS API vs C++ API



locomotor核心代码是严格的基于C++的最大灵活性。现在，这里有一个根本的问题，就是为什么我们甚至暴露C++的API。为什么不让一切都以行动为基础呢？在这种特殊情况下，ROS的消息系统可能无法充分覆盖C++提供的可扩展类的丰富性。 

尤其是，在消息/操作定义中，可能无法定义所有可能的错误代码。这样的一个列表将不包括仅在其他人的运动扩展可能需要的某些问题域中发生的特定错误。

通过围绕Locomor实现自定义状态机，可以为错误代码等定义自己的模式，并以与特定问题域兼容的方式发布反馈。这就是为什么一个特定的操作接口没有被烘焙到核心的locomotor类中，这样开发人员就可以定义/选择自己的操作定义。

 

## Locomove_base

### locomotor+move_base

locomove_base与含有状态机的locomotor进行配合来控制机器人

### Planners

#### Global Planner

因为他们是可互相替代的，所以任何全局规划器（nav_core或者av_core2）都可以使用。如果move_base使用nav_core规划器（标准版本），它将通过 `nav_core_adapter::GlobalPlannerAdapter2`被加载进locomove_base中。如果move_base使用nav_core2全局规划器那么就使用 `nav_core_adapter::GlobalPlannerAdapter`进行加载。locomove_base不需要适配器就可以直接使用nav_core2的全局规划器。

#### Local Planner

类似的，如果move_base使用nav_core2的局部规划器那么使用 `nav_core_adapter::LocalPlannerAdapter`作为适配器进行加载。而locomove_base可以直接使用而不需要适配器转换。

因为nav_core的局部规划器并不是向前兼容的，所以任何nav_core局部规划器都不能被locomove_base使用。另外，如果局部规划器是 `DWALocalPlanner`, `locomove_base`将会使用`DWBLocalPlanner`,因为他们的参数是向后兼容的。

#### Recovery Behaviors

恢复行为来自与nav_core，它的接口如下：
```$xslt
class RecoveryBehavior
{
public:
  virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) = 0;
  virtual void runBehavior() = 0;
}
```

为了使用RecoveryBehaviors， 我们必须能够获取一个 `Costmap2DROS`对象的原始指针。因此我们需要通过 `nav_core_adapter::CostmapAdapter`从 `nav_core2::Costmap`中获取指针

在 `move_base`中恢复行为是从参数服务器中的一个有序列表中被指定的（或者如果没有列表提供时使用默认的恢复行为）。每一个都是通过上面的接口进行初始化和保存

当有异常发生时，将有序的调用列表中 `runBehavior`方法来以此调用。具体逻辑如下图

#### Move Base State Machine

![Move base state machine](https://raw.githubusercontent.com/locusrobotics/robot_navigation/master/locomove_base/doc/state_machine.png)

##### Standard Operation

如果规划成功，状态机将直接通过左侧的流程完成，例如：

* 当收到一个初始目标点，那么将执行全局规划

* 当全局规划成功，接下来进行局部规划（即控制），同时也可能有全局规划发生

* 当到达目标点后，此次导航完成

##### Recovery from Failures

如果全局或者局部规划器 规划失败了，move_base将开始顺序执行恢复行为。当每个恢复行为执行完之后，会尝试重新规划，如果仍然规划失败，那么会执行下一个恢复行为，一直到所有的恢复行为都执行完仍然没能成功规划，那么此次当行失败

如果有一次规划成功以后，move_base将会回到标准流程。

* 如果全局规划失败了， move_base回在全局路径规划成功之后回到正常逻辑

*如果局部规划失败了，move_base会在每次恢复行为之后尝试规划新的全局路径， 只有在局部规划也成功以后才会回到正常流程中。







