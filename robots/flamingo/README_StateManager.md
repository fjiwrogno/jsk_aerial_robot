# Flamingo State Parameter Manager

## 概述

Flamingo State Parameter Manager 是一个ROS节点，用于管理flamingo（火烈鸟）机器人在不同操作模式之间的动态参数切换。该系统支持两种主要操作模式：

- **状态 0 (Air Mode)**: 空中模式，使用单向电机进行飞行
- **状态 1 (Water Mode)**: 水中模式，使用双向电机进行水下操作

## 系统架构

```
StateParameters.yaml ─→ State Parameter Manager ─→ ROS Parameter Server
                              ↑
                         state topic
                         (std_msgs/Int32)
```

## 配置文件

### StateParameters.yaml

包含所有状态配置的主配置文件，定义了：

- 电机参数（类型、PWM范围、推力曲线等）
- 控制器增益（PID参数）
- 控制分配矩阵
- 舵机控制器参数

每个状态都有完整的参数集合，支持完全不同的硬件配置。

## 使用方法

### 1. 启动参数管理器

```bash
roslaunch flamingo state_parameter_manager.launch
```

### 2. 手动切换状态

```bash
# 切换到空中模式
rosrun flamingo switch_state.py 0

# 切换到水中模式
rosrun flamingo switch_state.py 1
```

### 3. 程序化状态切换

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def switch_to_air_mode():
    pub = rospy.Publisher('state', Int32, queue_size=10)
    rospy.sleep(0.5)  # 等待发布器准备就绪

    msg = Int32()
    msg.data = 0  # 空中模式
    pub.publish(msg)

def switch_to_water_mode():
    pub = rospy.Publisher('state', Int32, queue_size=10)
    rospy.sleep(0.5)

    msg = Int32()
    msg.data = 1  # 水中模式
    pub.publish(msg)
```

### 4. C++ 状态切换

```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void switchState(ros::Publisher& pub, int state) {
    std_msgs::Int32 msg;
    msg.data = state;
    pub.publish(msg);

    ROS_INFO("Switching to state %d", state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_controller");
    ros::NodeHandle nh;

    ros::Publisher state_pub = nh.advertise<std_msgs::Int32>("state", 10);

    // 切换到空中模式
    switchState(state_pub, 0);

    // 切换到水中模式
    switchState(state_pub, 1);

    return 0;
}
```

## 参数映射

状态参数管理器会将YAML配置中的参数映射到相应的ROS参数：

### 电机参数

- `/rpy_controller_node/operating_mode`
- `/rpy_controller_node/min_pwm`
- `/rpy_controller_node/max_pwm`
- `/rpy_controller_node/pwm_conversion_mode`
- `/rpy_controller_node/motors/*/id`
- `/rpy_controller_node/motors/*/type`
- `/rpy_controller_node/motors/*/channel`

### 控制器增益

- `/rpy_controller_node/roll/pid_kp`
- `/rpy_controller_node/roll/pid_ki`
- `/rpy_controller_node/roll/pid_kd`
- `/rpy_controller_node/pitch/pid_kp`
- `/rpy_controller_node/pitch/pid_ki`
- `/rpy_controller_node/pitch/pid_kd`
- `/rpy_controller_node/yaw/pid_kp`
- `/rpy_controller_node/yaw/pid_ki`
- `/rpy_controller_node/yaw/pid_kd`

### 控制分配

- `/rpy_controller_node/torque_allocation_matrix_inv`
- `/rpy_controller_node/matrix_rows`
- `/rpy_controller_node/matrix_cols`

### 舵机控制器

- `/servo_bridge_node/gimbals/angle_sgn`
- `/servo_bridge_node/gimbals/angle_scale`
- `/servo_bridge_node/gimbals/zero_point_offset`

## 调试和监控

### 查看当前参数

```bash
# 查看所有rpy_controller_node参数
rosparam list | grep rpy_controller_node

# 查看特定参数值
rosparam get /rpy_controller_node/operating_mode
rosparam get /rpy_controller_node/motors
```

### 监控状态切换

```bash
# 监听状态切换话题
rostopic echo /state

# 查看参数管理器日志
rosnode info /state_parameter_manager
```

### 测试连续切换

```bash
# 启动自动切换测试（每10秒切换一次）
rosrun flamingo test_state_publisher.py
```

## 故障排除

### 常见问题

1. **YAML解析错误**
   - 检查StateParameters.yaml语法
   - 确保缩进正确使用空格而不是制表符

2. **参数未更新**
   - 确认状态参数管理器正在运行
   - 检查话题名称是否正确 (`/state`)
   - 查看节点日志输出

3. **找不到配置文件**
   - 检查文件路径: `$(find flamingo)/config/StateParameters.yaml`
   - 确认文件存在且可读

### 日志输出示例

```
[INFO] State Parameter Manager initialized with config: /path/to/StateParameters.yaml
[INFO] Loaded 2 state configurations
[INFO] State change detected: -1 -> 0
[INFO] Parameters updated for state 0
[INFO] Successfully switched to state 0
```

## 扩展和定制

### 添加新状态

1. 在`StateParameters.yaml`中添加新的状态配置
2. 确保包含所有必需的参数节
3. 重启参数管理器以加载新配置

### 添加新参数类型

1. 在`StateParameters.yaml`中定义新参数
2. 在`state_parameter_manager_new.cpp`中添加相应的更新函数
3. 在`updateParameters()`中调用新函数

### 自定义参数名称

修改`updateMotorParams()`, `updateControllerGains()`等函数中的参数路径以匹配你的节点名称。

## 性能考虑

- 参数更新是同步操作，通常在1秒内完成
- 状态切换不会中断正在运行的控制循环
- 配置文件在启动时加载一次，后续切换不需要重新读取文件
- 支持高频率状态切换（测试可达1Hz）

## 与现有代码的集成

该系统设计为最小化对现有代码的修改：

1. **无需修改控制代码** - 所有参数通过ROS参数服务器动态更新
2. **保持向后兼容** - 现有启动文件继续工作
3. **可选集成** - 可以独立运行或集成到现有系统中

## 文件结构

```
flamingo/
├── config/
│   └── StateParameters.yaml      # 状态配置文件
├── src/
│   └── state_parameter_manager_new.cpp  # 主要实现
├── scripts/
│   ├── switch_state.py          # 手动状态切换
│   └── test_state_publisher.py  # 自动测试切换
├── launch/
│   └── state_parameter_manager.launch   # 启动文件
├── CMakeLists.txt               # 构建配置
└── README_StateManager.md       # 本文档
```
