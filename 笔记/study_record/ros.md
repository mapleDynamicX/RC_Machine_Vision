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
