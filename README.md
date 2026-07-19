# easy_pid_controller_pkg

独立的 PID 控制器库，支持**位置式 P/I/D 三项完整控制**。  
提供标量和二维向量两种接口，专为机器人速度/位置闭环控制设计。

---

## 目录

1. [功能特性](#1-功能特性)
2. [快速开始](#2-快速开始)
3. [API 参考](#3-api-参考)
4. [参数说明](#4-参数说明)
5. [算法细节](#5-算法细节)
6. [跨任务安全](#6-跨任务安全)
7. [使用示例](#7-使用示例)
8. [编译与依赖](#8-编译与依赖)
9. [许可](#9-许可)

---

## 1. 功能特性

- **P/I/D 三项完整**：比例（P）、积分（I）、微分（D）独立计算
- **积分限幅（anti-windup）**：防止积分饱和引起的过冲
- **微分一阶低通滤波**：抑制噪声放大
- **死区（dead_zone）**：小误差输出零，防止抖动
- **输出总限幅**：限制最大输出值（如最大速度）
- **静摩擦补偿（down_machine_deadzone）**：低输出时拉升到阈值，防止进入下位机死区
- **标量和二维向量双接口**：单自由度用标量版本，xy 平移用向量版本
- **跨任务安全保护**：自动检测新任务并清零历史状态（见第 6 节）

---

## 2. 快速开始

### 2.1 创建 PID 控制器实例

```cpp
#include "easy_pid_controller.h"

// xy 方向平移 PID
pid_controller xy_controller(
    0.9,     // kp — 比例增益
    100,     // p_cut_val — P 项饱和阈值
    0.053,   // ki — 积分增益
    0.03,    // kd — 微分增益
    2,       // p_max
    2,       // i_max
    2,       // d_max
    0.5,     // total_max — 总输出限幅（最大速度）
    0.03,    // dead_zone — 死区
    0.6,     // alpha — 微分低通滤波系数
    30       // frequency — 控制频率（Hz）
);
```

### 2.2 控制循环

```cpp
while (running) {
    double error = compute_error();  // 你的误差计算

    controller.time();               // 1. 更新时间戳
    double output = controller.pd_output(error);  // 2. PID 计算

    apply_output(output);            // 3. 施加控制
    sleep_until_next_cycle();        // 4. 等待下一个周期
}
```

### 2.3 任务切换时重置

```cpp
// 新任务开始时，显式重置 PID 状态
controller.reset();
```

---

## 3. API 参考

### 构造函数

```cpp
pid_controller(
    double kp,           // 比例增益
    double p_cut_val,    // P 项饱和阈值
    double ki,           // 积分增益
    double kd,           // 微分增益
    double p_max,        // P 项输出限幅
    double i_max,        // I 项输出限幅
    double d_max,        // D 项输出限幅
    double total_max,    // 总输出限幅
    double dead_zone,    // 死区
    double d_fillter_alpha,  // 微分低通滤波系数（0~1）
    double frequency     // 控制频率（Hz）
);
```

### 成员函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `time()` | `void time()` | 更新时间戳。每次调用 `pd_output()` 前调用 |
| `pd_output()` | `double pd_output(const double& x)` | 标量 PID 输出（单自由度） |
| `pd_output()` | `Eigen::Vector2d pd_output(const Eigen::Vector2d& x)` | 二维向量 PID 输出（两个自由度并行） |
| `reset()` | `void reset()` | 重置全部状态：清零积分、微分、误差、时间戳 |
| `reset_integral()` | `void reset_integral()` | 仅重置积分项（保留微分历史） |

---

## 4. 参数说明

### 4.1 PID 核心参数

| 参数 | 典型范围 | 含义 | 调大后果 | 调小后果 |
|------|---------|------|----------|----------|
| `kp` | 0.1 ~ 2.0 | 比例增益 | 响应变快，但容易震荡或过冲 | 响应慢，但更稳定 |
| `ki` | 0.01 ~ 0.2 | 积分增益 | 消除稳态误差更快，但可能积分饱和引起大幅过冲 | 消除稳态误差慢 |
| `kd` | 0.005 ~ 0.1 | 微分增益 | 阻尼更强，抑制震荡 | 可能震荡 |
| `total_max` | 0.1 ~ 1.0 | 总输出限幅 | 控制力度更大，可能超调 | 更安全，但到位慢 |
| `dead_zone` | 0.01 ~ 0.1 | 死区 | 容忍更大误差不动作 | 过于敏感，可能原地抖动 |

### 4.2 限幅参数

| 参数 | 说明 |
|------|------|
| `p_cut_val` | **P 项饱和阈值（核心设计意图见下方详解）**。当 \|error\| > p_cut_val 时，P 项直出 p_max；否则 P = error × kp。 |
| `p_max` | P 项输出限幅 |
| `i_max` | I 项输出限幅（积分 anti-windup） |
| `d_max` | D 项输出限幅 |
| `total_max` | P+I+D 求和后的总输出限幅（最外层） |

### 4.2a p_cut_val 的设计意图详解

`p_cut_val` 不是一个普通的"防饱和"参数，它承担了**上下位机加速控制权切换**的关键角色。

**背景：** 今年的下位机（达妙驱动板）具备自主的加速度控制和速度规划能力。这意味着：
- 在任务中，下位机有提出过建议“p直接拉满输出”
，这便是预留项的使命
- 在另一些任务中，上位机需要精细的 PID 闭环控制

**`p_cut_val` 的妙用：**

| 场景 | p_cut_val 设值 | 效果 |
|------|---------------|------|
| **上位机全权闭环** | 100（大值） | 比例项全程线性输出 `kp × error`，上位机精确控制 |
| **下位机自主加减速** | 0.1 ~ 0.3（小值） | 误差稍大时 P 项直接饱和到 `p_max`，相当于"油门踩死"，由下位机自行规划速度曲线 |

**为什么保留这个参数？**
- 不依赖 `p_cut_val` 时，设大即可（行为退化为纯线性 P）
- 需要时设小，立即获得下位机自主加减速能力
- 零运行时开销（仅一次比较）
- 可在 PID 构造时固定，也可运行时通过 `reset()` + 重建控制器切换模式

**注意：** `p_cut_val` 在构造函数中会自动截断：
```cpp
p_cut_val = p_cut_val > p_max / kp ? p_max / kp : p_cut_val;
```
即 `p_cut_val` 不会超过 `p_max / kp`，因为这会导致 P 项永远达不到 `p_max`（无意义）。

### 4.3 滤波与补偿

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `d_fillter_alpha` | 0.6 | 微分低通滤波系数（0~1），越大滤波越弱、响应越快 |
| `down_machine_deadzone` | 0.1 | 静摩擦补偿。PID 输出小于此值时拉升到此值，确保低误差下机器人能动起来。**硬编码在 .cpp 中** |
| `max_dt` | `1/freq * 1.2` | dt 最大值保护，防止长时间未调用时积分/微分突变。自动计算 |
| `MAX_STALE_DT` | 0.5 秒 | 跨任务检测阈值。距离上次调用超过此值视为新任务，自动清零（见第 6 节） |

---

## 5. 算法细节

### 5.1 计算公式

```
P 项：p_out = x * kp（超过 p_cut_val 时饱和到 p_max）
I 项：i_val += ki * error * dt（梯形积分，限幅到 ±i_max）
D 项：d_val = (dx/dt) * kd（一阶低通滤波 + 限幅到 ±d_max）

P + I + D 求和 → 限幅到 ±total_max
     → 如果小于 down_machine_deadzone → 拉升到 down_machine_deadzone（克服静摩擦）
```

### 5.2 标量接口执行流程

```
pd_output(x)
├── 计算 dt = current_t - last_error_t（上限 max_dt）
├── 如果 dt > MAX_STALE_DT(0.5s) → reset()，只输出 P 项（新任务保护）
├── 如果 |x| < dead_zone → 清零状态，输出 0
├── 计算 p_out = p_function(x)
├── 计算 d_out = d_fillter(x - last_error, last_d_val, dt)
├── 计算 i_out = i_function(x, dt)
├── 更新状态 (last_error, last_d_val, last_i_val, last_error_t)
├── 求和 + 静摩擦补偿 + 输出限幅
└── 返回输出值
```

### 5.3 二维向量接口执行流程

对误差向量的每个分量独立执行上述标量流程，使用各自独立的历史状态数组。

---

## 6. 跨任务安全

### 问题描述

PID 控制器的 **积分累积**（`last_i_val`）和 **微分历史**（`last_error`, `last_d_fillter_val`）在上一个任务结束后仍然保留。如果同一个 PID 实例被复用于下一个任务，残留的历史值会导致：

- 起始时刻输出跳变（错误微分项）
- 积分饱和导致大幅过冲
- 需要很长时间才能"退饱和"

### 解决方案

本控制器提供**两层防护**：

| 层级 | 机制 | 触发条件 | 效果 |
|------|------|---------|------|
| **1. 自动检测** | `pd_output()` 内部判断 dt | dt > `MAX_STALE_DT`（0.5s） | 自动调用 `reset()` + 只输出 P 项 |
| **2. 显式重置** | 调用 `reset()` 方法 | 任务开始/结束时由用户调用 | 完全清零全部状态 |

> 两层防护独立工作，建议**同时启用**以获得最大安全性。

### 推荐用法

```cpp
// 每次新任务开始时
controller.reset();      // 显式清零
controller.time();       // 更新时间戳

while (task_running) {
    controller.time();
    double out = controller.pd_output(error);
    // ...
}
```

---

## 7. 使用示例

### 示例 1：单自由度角度控制

```cpp
pid_controller angle_controller(1.0, 10, 0.05, 0.02, 3, 3, 3, 0.5, 0.02, 0.6, 50);

void control_loop() {
    angle_controller.time();
    double angle_error = target_angle - current_angle;
    double torque = angle_controller.pd_output(angle_error);
    motor_set_torque(torque);
}
```

### 示例 2：两自由度 xy 平移控制

```cpp
pid_controller xy_controller(0.9, 100, 0.053, 0.03, 2, 2, 2, 0.5, 0.03, 0.6, 30);

void control_loop() {
    xy_controller.time();
    Eigen::Vector2d error(target_x - current_x, target_y - current_y);
    Eigen::Vector2d output = xy_controller.pd_output(error);
    robot_set_velocity(output.x(), output.y());
}
```

### 示例 3：类成员 + 多任务对齐控制（贴近真实 ROS 用法）

这是从 `align_controller.cpp` 中提炼的真实调用模式——两个 PID 控制器（xy 平移 + omega 旋转）作为类成员，在 ROS 回调中驱动闭环控制。

```cpp
#include "easy_pid_controller.h"

class RobotController {
private:
    // 两个 PID 实例：一个控制 xy 平移（二维向量），一个控制 omega 旋转（标量）
    pid_controller xy_controller;
    pid_controller omega_controller;

    Eigen::Vector2d last_xy_err;
    double last_omega_err;

public:
    RobotController()
        // ===== 构造时传入完整参数 =====
        : xy_controller(pid_controller(
              0.9,     // kp
              100,     // p_cut_val
              0.053,   // ki
              0.03,    // kd
              2,       // p_max
              2,       // i_max
              2,       // d_max
              0.5,     // total_max
              0.03,    // dead_zone
              0.6,     // alpha
              30       // frequency (Hz)
          )),
          omega_controller(pid_controller(
              0.8, 0.07, 0.02, 1, 1, 1, 0.5, 0.03, 0.6, 30
          )) {}

    // ===== 每个控制周期调用的回调 =====
    void control_step(Eigen::Vector2d xy_error, double omega_error) {
        // 1. 必须先调用 time()，更新内部时间戳，供 pd_output() 计算 dt
        this->xy_controller.time();
        this->omega_controller.time();

        // 2. 计算 PID 输出
        //    xy_controller.pd_output(Eigen::Vector2d) → 返回 Eigen::Vector2d
        //    omega_controller.pd_output(double)      → 返回 double
        Eigen::Vector2d xy_output = this->xy_controller.pd_output(xy_error);
        double omega_output = this->omega_controller.pd_output(omega_error);

        // 3. 保存误差供日志/调试
        this->last_xy_err = xy_error;
        this->last_omega_err = omega_error;

        // 4. 下发控制（示例）
        send_motor_command(xy_output.x(), xy_output.y(), omega_output);
    }

    // ===== 新任务开始时：清零 PID 历史状态 =====
    void start_new_task() {
        // 防止上一个任务的积分/微分残留到新任务
        this->xy_controller.reset();
        this->omega_controller.reset();

        ROS_INFO("PID states reset for new task");
    }

    // ===== 仅需清零积分时（可选） =====
    void clear_integral_only() {
        this->xy_controller.reset_integral();
    }
};
```

**使用时的完整生命周期：**

```cpp
RobotController robot;

// ---- 第一次对齐任务 ----
robot.start_new_task();        // PID 清零
while (robot.is_alive()) {
    auto error = robot.compute_xy_omega_error();
    robot.control_step(error.first, error.second);
    ros::Rate(30).sleep();
}

// ---- 一段时间后，第二次对齐任务 ----
robot.start_new_task();        // PID 再次清零（关键！）
while (robot.is_alive()) {
    auto error = robot.compute_xy_omega_error();
    robot.control_step(error.first, error.second);
    ros::Rate(30).sleep();
}
```

> ⚠️ **如果不调用 `reset()`**，第二次对齐任务启动时 `last_i_val` 中残留着第一次任务的积分值，会导致起始时刻输出跳变。虽然 `pd_output()` 内部有自动检测保护（dt > 0.5s）并进行清零，显式调用 `reset()` 是最安全的做法。

---

## 8. 编译与依赖

### 依赖

| 依赖 | 用途 |
|------|------|
| Eigen3 | 向量运算支持 |
| C++17 | `std::chrono`、`std::clamp` 等 |

### 使用方式：直接复制源码

PID 控制器就两个文件，**不需要单独编译这个包**，直接复制到你的包目录里用：

```
你的包/
├── include/
│   └── easy_pid_controller.h      ← 复制过来
├── src/
│   ├── easy_pid_controller.cpp    ← 复制过来
│   └── your_node.cpp
└── CMakeLists.txt
```

**在 CMakeLists.txt 中添加：**

```cmake
include_directories(include)                              # 确保能找到头文件
add_library(your_core src/easy_pid_controller.cpp)        # 先做成库
target_link_libraries(your_node your_core)                # 再链接到你的节点
```

然后直接 `#include` 使用：

```cpp
#include "easy_pid_controller.h"

pid_controller my_pid(0.9, 100, 0.053, 0.03, 2, 2, 2, 0.5, 0.03, 0.6, 30);
my_pid.time();
double output = my_pid.pd_output(some_error);
```

---

## 9. 许可

MIT License.  
Copyright (c) 2026 robopioneer
