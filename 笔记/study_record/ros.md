# ROS

## ROS1 中，指定 `rosbag` 文件的发布频率

```
rosbag play -r <倍率> your_bag.bag
```

> - **作用​**​：对整个 bag 文件的播放速度进行缩放。
> - ​**​示例​**​：
>   - `-r 0.5`：以原始速度的 50% 慢速播放。
>   - `-r 2.0`：以原始速度的 2 倍快速播放。
> - ​**​原理​**​：修改所有消息的时间戳间隔，实现整体加速/减速。

示例：播放`example.bag`，并以2倍速播放， rosbag play -r 2.0 example.bag

## 回调函数

当多个消息同时到达时，​**​回调函数默认不会并行执行​**​

### **多线程处理​**​

```cpp
ros::AsyncSpinner spinner(N); // N = 线程数（建议与回调数量匹配）
spinner.start();
ros::waitForShutdown();
```

共享数据需用​**​互斥锁（`std::mutex`）​**​ 保护

## 类和对象

**发布者（Publisher）**

> 发布者只需要一个话题名称和消息类型，它负责发布消息。当我们调用发布者的`publish()`方法时，通常是在类的某个成员函数中主动调用，比如在定时器回调或接收到订阅消息后的处理函数中。因此，发布者本身不需要绑定回调函数，所以不需要特别绑定类对象（但发布者作为类的成员变量，自然属于该对象）

**订阅者（Subscriber）**

> 订阅者需要指定一个回调函数，当接收到消息时，ROS会调用这个回调函数。如果我们在类中定义订阅者，我们通常希望回调函数能够访问类的成员（变量或函数）。因此，我们需要将这个回调函数绑定到类的实例（对象）上。如果不这样做，回调函数将无法访问类的非静态成员（因为非静态成员需要通过对象来访问）。

**举例**

```cpp
// 构造函数初始化
MyNode::MyNode() {
    // 1. 发布者初始化 (简单初始化)
    pub = nh.advertise<std_msgs::String>("output_topic", 10);
    // 2. 订阅者初始化 - 关键绑定步骤！
    sub = nh.subscribe(
        "input_topic",              // 话题名
        10,                         // 队列长度
        &MyNode::subCallback,       // 成员函数指针
        this                        // ★★★ 绑定当前对象 ★★★
    );
}
// 订阅回调函数
void MyNode::subCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());

    // 使用发布者（直接通过this调用）
    std_msgs::String new_msg;
    new_msg.data = "Echo: " + msg->data;
    pub.publish(new_msg);  // ✓ 可直接访问成员pub
}
```

# ROS1 定时器详解：`nh.createTimer`

```cpp
timer_ = nh.createTimer(ros::Duration(1 / freq_localization_),
                        std::bind(&GlobalLocalization::threadLocalization, this),
                        false, true);
```

这是ROS1中创建定时器的标准方法，功能是​**​以指定频率周期性地调用成员函数​

### 1. 时间间隔（`ros::Duration`）

```cpp
ros::Duration(1 / freq_localization_)​
```

> - **作用​**​：定义定时器触发的时间间隔（单位：秒）
> - ​**​计算​**​：
>   - `freq_localization_`：期望的调用频率（单位：Hz）
>   - `1 / freq_localization_`：计算两次调用间的时间间隔
> - ​**​示例​**​：
>   - 若 `freq_localization_ = 10.0` (10Hz)，则间隔为0.1秒
>   - 若 `freq_localization_ = 1.0` (1Hz)，则间隔为1.0秒

### 2. 回调函数（`std::bind`）

```cpp
std::bind(&GlobalLocalization::threadLocalization, this)
```

> - ​**​作用​**​：绑定定时器触发时要调用的成员函数
> - ​**​解析​**​：
>   - `&GlobalLocalization::threadLocalization`：成员函数指针
>   - `this`：绑定当前对象实例
> - ​**​特殊说明​**​：
>   - 这种写法表示`threadLocalization`函数​**​不需要任何参数​**​
>   - 如果需要带参数（如默认的`TimerEvent`），需使用占位符：

​**​普通函数​**​

```cpp
// 函数名可自动转换为函数指针
void freeFunc(int); 
subscribe("topic", freeFunc); // ✅ 合法
```

**成员函数​**​：

```cp
class MyClass {
public:
   void memberFunc(int);
};
// 成员函数隐含`this`参数
MyClass obj;
subscribe("topic", obj.memberFunc); // ❌ 错误！不能直接使用
//为何需要 & 和 this
_cloud_sub = _nh.subscribe(_cloud_sub_name, 10, 
                         &registration2::cloudCallback, // 必须加&
                         this);                         // 必须提供对象实例
```

> - ​**​类型特殊性​**​：
>   
>   - 成员函数指针类型为 `ReturnType (Class::*)(Params...)`（类作用域限定）
>   - 普通函数指针类型为 `ReturnType (*)(Params...)`（全局作用域）

### 3. 一次性执行标志（布尔值）

```cpp
false
```

> - **作用​**​：控制定时器触发模式
> - ​**​值解析​**​：
>   - `true`：只执行一次（类似`setTimeout`）
>   - `false`：周期性执行（类似`setInterval`）← ​**​本例使用模式​**​
> - ​**​应用场景​**​：
>   - `true`：适用于初始化后只需要执行一次的任务
>   - `false`：适用于需要持续周期性处理的任务

### 4. 自动启动标志（布尔值）

```cpp
true
```

> - **作用​**​：控制定时器创建后是否立即启动
> - ​**​值解析​**​：
>   - `true`：创建后立即启动（默认启用）
>   - `false`：需要手动调用`timer.start()`启动
> - ​**​应用场景​**​：
>   - `true`：默认情况，创建即工作
>   - `false`：需要等待其他条件满足后才启动的场景

## 定时器操作完整方法

### 创建后操作定时器：

```cpp
timer_.stop();      // 停止定时器
timer_.start();     // 重启定时器
timer_.setPeriod(ros::Duration(0.2));  // 动态修改时间间隔
```

## 定时器的生命周期管理

### 作用域要求

```cpp
class GlobalLocalization {
private:
    ros::Timer timer_;  // 必须作为成员变量！不能是局部变量
public:
    void init() {
        // ❌ 错误：局部变量会导致回调无法持续
        // ros::Timer local_timer = nh.createTimer(...);

        // ✅ 正确：作为成员变量
        timer_ = nh.createTimer(...);
    }
};
```

> ### 生命周期注意事项
> 
> - ​**​保持有效​**​：定时器对象需要在所有回调执行期间保持有效
> - ​**​节点关闭​**​：定时器会随着节点关闭自动销毁
> - ​**​提前销毁​**​：可通过`timer_.stop()`停止并释放资源

### ROS1 中的服务（Service）消息详解

#### 1. 服务消息的作用

> 服务消息是 ROS1 中实现 ​**​同步请求/响应（Request/Response）通信机制​**​的核心组件，与异步的发布/订阅（Topic）机制形成互补：
> 
> 1. ​**​同步通信​**​：客户端发出请求后阻塞等待服务端响应
> 2. ​**​精确控制​**​：适用于需要确认执行结果的操作
> 3. ​**​任务型交互​**​：完成特定任务后返回结果
> 4. ​**​避免轮询​**​：替代通过 Topic 不断查询状态的低效方式

#### 2. 通信流程

![](/home/maple/笔记/images/2025-07-10-10-37-07-ros1服务.png)

### **Eigen (强烈推荐)**

> - **核心优势​**​：ROS和PCL默认依赖的线性代数库，无额外安装成本
> - ​**​特点​**​：
>   - 头文件库（无需编译）
>   - 支持矩阵运算、分解、几何变换等（如SVD、QR分解）
>   - 与ROS/PCL深度集成
> - ​**​ROS1集成​**​：
>   - `tf`和`tf2`直接使用Eigen处理坐标变换
>   - `geometry_msgs/Pose`与Eigen转换工具：`tf2_eigen`
> - ​**​PCL集成​**​：
>   - 点云类型（如`pcl::PointXYZ`）与Eigen互相转换
>   - 特征值计算（如法线估计）、ICP配准基于Eigen

## .toSec()

```cpp
ros::Time start = ros::Time::now();
// 执行某些操作
ros::Time end = ros::Time::now();
ros::Duration duration = end - start;
double duration_sec = duration.toSec(); // 转换为秒
```
