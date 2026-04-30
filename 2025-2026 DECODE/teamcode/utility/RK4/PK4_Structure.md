# RK4 跑打算法架构文档

## 目录
- [1. 系统概述](#1-系统概述)
- [2. 核心原理](#2-核心原理)
  - [2.1 物理模型](#21-物理模型)
  - [2.2 RK4数值积分](#22-rk4数值积分)
  - [2.3 双层迭代求解](#23-双层迭代求解)
- [3. 程序架构](#3-程序架构)
  - [3.1 类依赖关系](#31-类依赖关系)
  - [3.2 核心类详解](#32-核心类详解)
- [4. 计算流程](#4-计算流程)
  - [4.1 完整求解流程](#41-完整求解流程)
  - [4.2 AutoSelect优化流程](#42-autoselect优化流程)
- [5. 关键算法](#5-关键算法)
  - [5.1 RK4积分步骤](#51-rk4积分步骤)
  - [5.2 二分搜索](#52-二分搜索)
  - [5.3 旋转角修正](#53-旋转角修正)
- [6. 性能优化](#6-性能优化)
- [7. 使用建议](#7-使用建议)

## 1. 系统概述

RK4跑打算法是一个用于FTC（FIRST Tech Challenge）机器人射击系统的精确计算系统。该系统考虑了以下关键因素：

- **空气阻力**：使用非线性阻力模型 F = -k·vⁿ
- **小车运动**：考虑小车的移动速度对发射的影响
- **目标运动**：预测目标在飞行时间内的位置
- **高度差**：处理炮口与目标的高度差异

系统采用四阶龙格-库塔（Runge-Kutta 4th order）数值积分方法，能够精确模拟抛射体轨迹，并通过双层迭代算法求解最优发射参数。

### 主要特点

1. **高精度**：RK4积分提供O(h⁴)精度的数值解
2. **实时性**：优化后的算法适合实时控制
3. **鲁棒性**：多层验证确保解的可靠性
4. **灵活性**：支持多种模式和参数配置

## 2. 核心原理

### 2.1 物理模型

#### 运动方程

抛射体在三维空间中的运动由以下微分方程描述：

```
dx/dt = vx
dy/dt = vy
dz/dt = vz
dvx/dt = -kx/m · v · vx
dvy/dt = -kx/m · v · vy
dvz/dt = -kx/m · v · vz + ky/m · v - g
```

其中：
- `(x, y, z)`：位置坐标（米）
- `(vx, vy, vz)`：速度分量（米/秒）
- `v = √(vx² + vy² + vz²)`：总速度
- `kx`：空气阻力系数
- `ky`：空气升力系数
- `m`：小球质量（公斤）
- `g`：重力加速度（9.81 m/s²）

#### 阻力和升力模型

采用线性阻力模型和升力模型：

```
F_drag = -kx · v · v̂
F_lift = ky · v · ẑ
```

- `F_drag`：阻力，方向与速度相反
- `F_lift`：升力，方向竖直向上
- `kx`：阻力系数，范围 -0.1~0.1
- `ky`：升力系数，范围 -0.5~0.5

#### 初始条件

考虑小车运动和发射角度：

```
vx₀ = v₀ · cos(θ) · cos(φ) + robotVx
vy₀ = v₀ · cos(θ) · sin(φ) + robotVy
vz₀ = v₀ · sin(θ)
```

其中：
- `v₀`：发射初速度（米/秒）
- `θ`：仰角（弧度，0-π/2）
- `φ`：旋转角（弧度，-π~π，与小车x轴夹角）
- `robotVx`：小车x方向速度
- `robotVy`：小车y方向速度

### 2.2 RK4数值积分

#### 算法原理

四阶龙格-库塔方法是一种显式数值积分方法，通过计算四个斜率的加权平均来获得高精度的解：

```
k₁ = f(t, y)
k₂ = f(t + h/2, y + h·k₁/2)
k₃ = f(t + h/2, y + h·k₂/2)
k₄ = f(t + h, y + h·k₃)

y(t+h) = y(t) + (k₁ + 2k₂ + 2k₃ + k₄) · h/6
```

#### 实现细节

在抛射体轨迹仿真中，状态向量为：

```
y = [x, y, z, vx, vy, vz]
f(t, y) = [vx, vy, vz, ax, ay, az]
```

每次RK4步骤：

1. **计算k₁**：在当前状态计算导数
2. **计算k₂**：在t + h/2时刻，使用k₁预测的状态计算导数
3. **计算k₃**：在t + h/2时刻，使用k₂预测的状态计算导数
4. **计算k₄**：在t + h时刻，使用k₃预测的状态计算导数
5. **更新状态**：使用加权平均更新状态

#### 时间步长选择

- 默认步长：0.01秒
- 最大飞行时间：5.0秒
- 最大步数：500步

步长权衡：
- 较小步长：精度高，但计算量大
- 较大步长：计算快，但精度低

### 2.3 双层迭代求解

#### 问题定义

给定目标位置 `(targetX, targetY)` 和小车状态 `(robotVx, robotVy)`，求解：
- 仰角 `θ`
- 初速度 `v₀`（可选）
- 旋转角 `φ`

#### 双层迭代结构

**外层迭代**：优化旋转角 φ
1. 根据目标位置计算初始旋转角
2. 使用内层迭代求解仰角
3. 模拟轨迹，计算落点
4. 根据落点误差修正旋转角
5. 重复直到收敛

**内层迭代**：求解仰角 θ 或初速度 v₀
1. 使用二分搜索在有效范围内搜索
2. 对于每个候选值，模拟轨迹
3. 检查是否达到目标高度
4. 选择最优解

#### 收敛条件

- 旋转角变化小于容差（默认0.5度）
- 达到最大迭代次数（默认10次）
- 二分搜索达到精度要求（默认0.001弧度）

## 3. 程序架构

### 3.1 类依赖关系

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层（用户接口）                       │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐                                │
│  │ AutoSelect         │◄───── 用户调用                   │
│  │ [自动参数选择]      │                                │
│  └──────────┬────────┘                                │
│             │                                           │
└─────────────┼───────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────┐
│                    求解层（核心算法）                       │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐                                │
│  │ Solver             │◄───── AutoSelect调用              │
│  │ [双层迭代求解器]     │                                │
│  └──────────┬────────┘                                │
│             │                                           │
│             ├──────────────────────────────────────┐        │
│             │                              │        │
│             ▼                              ▼        │
│  ┌─────────────────────┐         ┌─────────────────────┐ │
│  │ TrajectorySimulator│         │ TargetPredictor    │ │
│  │ [轨迹仿真器]        │         │ [目标预测器]        │ │
│  └──────────┬────────┘         └─────────────────────┘ │
│             │                                           │
│             ├──────────────────────────────────────┐        │
│             │                              │        │
│             ▼                              ▼        │
│  ┌─────────────────────┐         ┌─────────────────────┐ │
│  │ ProjectileDynamics │         │ RobotState         │ │
│  │ [运动微分方程]      │         │ [机器人状态]        │ │
│  └──────────┬────────┘         └─────────────────────┘ │
│             │                                           │
│             ├──────────────────────────────────────┐        │
│             │                              │        │
│             ▼                              ▼        │
│  ┌─────────────────────┐         ┌─────────────────────┐ │
│  │ ProjectileState    │         │ ProjectileParameters│ │
│  │ [小球状态]         │         │ [物理参数]          │ │
│  └─────────────────────┘         └─────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 核心类详解

#### ProjectileParameters（物理参数）

**职责**：存储和管理物理系统参数

**主要字段**：
```java
public double v0;        // 初速度（m/s）
public double kx;        // 阻力系数
public double ky;        // 升力系数
public double m;         // 小球质量（kg）
public double g;         // 重力加速度（m/s²）
public double deltaH;     // 高度差（m）
public double thetaMax;   // 最大仰角（弧度）
public double thetaMin;   // 最小仰角（弧度）
```

**设计模式**：Builder模式（withXxx方法）

**默认值**：
- v₀ = 10.0 m/s
- kx = 0.015
- ky = 0.0
- m = 0.1 kg
- g = 9.81 m/s²
- deltaH = 0.5 m
- θ_min = 45°
- θ_max = 65°

#### ProjectileState（小球状态）

**职责**：表示小球在某一时刻的运动状态

**主要字段**：
```java
public double x, y, z;      // 位置坐标（m）
public double vx, vy, vz;   // 速度分量（m/s）
public double time;          // 时间（s）
```

**辅助方法**：
- `getSpeed()`：计算总速度
- `getHorizontalSpeed()`：计算水平速度
- `getHorizontalDistance()`：计算水平距离

#### ProjectileDynamics（运动微分方程）

**职责**：计算抛射体的加速度

**核心方法**：
```java
public static double[] computeDerivatives(
    ProjectileState state, 
    ProjectileParameters params
)
```

**计算逻辑**：
1. 计算总速度：`v = √(vx² + vy² + vz²)`
2. 计算阻力分量（方向与速度相反）：
   - `dragForceX = -kx · v · vx`
   - `dragForceY = -kx · v · vy`
   - `dragForceZ = -kx · v · vz`
3. 计算升力分量（方向竖直向上）：
   - `liftForceZ = ky · v`
4. 计算加速度分量：
   - `ax = dragForceX / m`
   - `ay = dragForceY / m`
   - `az = (dragForceZ + liftForceZ) / m - g`
5. 返回导数向量：`[vx, vy, vz, ax, ay, az]`

#### TrajectorySimulator（轨迹仿真器）

**职责**：使用RK4方法仿真抛射体轨迹

**核心方法**：
```java
public TrajectoryResult simulate(
    double turretPhi,      // 旋转角
    double theta,          // 仰角
    double startX, startY, startZ,  // 起始位置
    double robotVx, robotVy,        // 小车速度
    ProjectileParameters params         // 物理参数
)
```

**仿真流程**：
1. 计算初始速度（考虑小车运动）
2. 初始化状态
3. 循环执行RK4步骤
4. 检测穿越目标高度
5. 插值计算精确落点
6. 返回轨迹结果

**关键特性**：
- 检测两次穿越目标高度（上升和下落）
- 使用线性插值提高精度
- 防止无限循环（最大步数限制）

**返回结果**：
```java
public static class TrajectoryResult {
    public final double landingX, landingY, landingZ;  // 落点坐标
    public final double flightTime;                     // 飞行时间
    public final boolean reachedTargetHeight;           // 是否达到目标高度
    public final double theta, turretPhi;              // 发射参数
}
```

#### RobotState（机器人状态）

**职责**：表示机器人的位置和速度

**主要字段**：
```java
public double x, y;     // 位置（m）
public double vx, vy;   // 速度（m/s）
```

**使用场景**：
- 作为求解器的输入
- 考虑小车运动对发射的影响

#### TargetPredictor（目标预测器）

**职责**：预测目标在未来的位置

**核心方法**：
```java
public static TargetState predict(
    double x0, y0, vx, vy,  // 初始状态
    double t                   // 预测时间
)
```

**预测模型**：匀速直线运动
```
x(t) = x₀ + vx · t
y(t) = y₀ + vy · t
```

**使用场景**：
- 预测目标在飞行时间内的位置
- 提高移动目标的命中率

#### Solver（求解器）

**职责**：双层迭代求解最优发射参数

**核心方法**：
```java
// 模式1：固定初速度，求解仰角
public SolverResult solve(
    double relativeX, relativeY,  // 目标位置
    double robotVx, robotVy,      // 小车速度
    double v0                     // 固定初速度
)

// 模式2：固定仰角，求解初速度
public SolverResult solve(
    double relativeX, relativeY,  // 目标位置
    double robotVx, robotVy,      // 小车速度
    double theta,                  // 固定仰角
    String mode                   // "Vel"
)
```

**求解流程**：

**外层迭代**（优化旋转角φ）：
1. 计算初始旋转角：`φ₀ = atan2(dy, dx)`
2. 循环（最多10次）：
   a. 使用内层迭代求解仰角/初速度
   b. 模拟轨迹，获取飞行时间
   c. 预测目标位置：`target = initial + v_target · flightTime`
   d. 计算地面角度：`groundAngle = atan2(dy, dx)`
   e. 计算修正量：`correction = (robotVy·cos(groundAngle) - robotVx·sin(groundAngle)) / v_horizontal`
   f. 更新旋转角：`φ = groundAngle - correction`
   g. 检查收敛：`|φ_new - φ_old| < 0.5°`

**内层迭代**（二分搜索）：
1. 设置搜索范围（仰角45-65度，初速度0-10 m/s）
2. 循环（最多30次）：
   a. 计算中点值
   b. 模拟轨迹
   c. 检查是否达到目标高度
   d. 调整搜索范围
   e. 检查收敛

**返回结果**：
```java
public static class SolverResult {
    public final double turretPhi;    // 旋转角（弧度）
    public final double theta;        // 仰角（弧度）
    public final double v0;           // 初速度（m/s）
    public final double flightTime;    // 飞行时间（s）
    public final boolean success;      // 是否成功
    public final String message;       // 消息
    
    public double getTurretPhiDegrees();
    public double getThetaDegrees();
}
```

#### AutoSelect（自动参数选择）

**职责**：自动选择最优初速度和仰角组合

**核心方法**：
```java
public AutoSelectResult Select(
    double relativeX, relativeY,  // 目标位置
    double robotVx, robotVy,      // 小车速度
    double initialV0,              // 初始初速度
    double initialTheta            // 初始仰角
)
```

**优化策略**（合并模式）：

1. **尝试初始参数**：
   - 使用初始初速度求解仰角
   - 使用初始仰角求解初速度
   - 记录初始解

2. **二分法同时优化**：
   - 扩大搜索范围（初速度±1.5m/s，仰角±10°）
   - 循环50次：
     a. 固定初速度，优化仰角
     b. 固定仰角，优化初速度
     c. 计算落点误差
     d. 综合误差 = 落点误差 + 0.01×参数调整量
     e. 更新最优解
     f. 缩小搜索范围
   - 最终验证误差 < 0.1m

3. **返回最优解**：
   - 如果优化成功，返回优化后的解
   - 否则返回初始解

**返回结果**：
```java
public static class AutoSelectResult {
    public final double theta;        // 仰角（弧度）
    public final double v0;           // 初速度（m/s）
    public final double turretPhi;    // 旋转角（弧度）
    public final boolean success;      // 是否成功
    public final String message;       // 消息
    
    public double getThetaDegrees();
    public double getTurretPhiDegrees();
}
```

## 4. 计算流程

### 4.1 完整求解流程

```
用户输入
    │
    ▼
┌─────────────────────────────────────┐
│ 1. 输入参数                     │
│    - 目标位置 (targetX, targetY) │
│    - 小车速度 (robotVx, robotVy) │
│    - 初始初速度 (initialV0)      │
│    - 初始仰角 (initialTheta)     │
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ 2. AutoSelect.Select()          │
│    - 尝试初始参数组合            │
│    - 启动优化过程                │
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ 3. 优化循环（50次）             │
│    a. 固定初速度，优化仰角       │
│       └─ Solver.solve(v0)       │
│          └─ 二分搜索仰角         │
│             └─ 模拟轨迹         │
│                └─ RK4积分       │
│                   └─ 计算误差   │
│    b. 固定仰角，优化初速度       │
│       └─ Solver.solve(theta)     │
│          └─ 二分搜索初速度       │
│             └─ 模拟轨迹         │
│                └─ RK4积分       │
│                   └─ 计算误差   │
│    c. 更新最优解                │
│    d. 缩小搜索范围              │
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ 4. 最终验证                     │
│    - 模拟最优参数轨迹            │
│    - 计算落点误差               │
│    - 验证误差 < 0.1m           │
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ 5. 返回结果                     │
│    - 仰角 (theta)                │
│    - 初速度 (v0)                 │
│    - 旋转角 (turretPhi)          │
│    - 成功标志 (success)           │
│    - 消息 (message)              │
└──────────────┬──────────────────┘
               │
               ▼
         用户使用结果
```

### 4.2 AutoSelect优化流程

```
开始优化
    │
    ▼
┌─────────────────────────────────────┐
│ 初始化搜索范围                    │
│ - v0: [initialV0-1.5, initialV0+1.5] │
│ - theta: [initialTheta-10°, initialTheta+10°] │
└──────────────┬──────────────────┘
               │
               ▼
         ┌─────┴─────┐
         │  循环50次  │
         └─────┬─────┘
               │
               ▼
    ┌──────────┴──────────┐
    │ 1. 固定v0，优化theta │
    │    - 计算v0中点      │
    │    - Solver.solve(v0) │
    │    - 获取theta       │
    │    - 模拟轨迹        │
    │    - 计算误差        │
    │    - 更新最优解      │
    │    - 调整theta范围  │
    └──────────┬──────────┘
               │
               ▼
    ┌──────────┴──────────┐
    │ 2. 固定theta，优化v0 │
    │    - 计算theta中点   │
    │    - Solver.solve(theta)│
    │    - 获取v0          │
    │    - 模拟轨迹        │
    │    - 计算误差        │
    │    - 更新最优解      │
    │    - 调整v0范围    │
    └──────────┬──────────┘
               │
               ▼
    ┌──────────┴──────────┐
    │ 3. 缩小搜索范围     │
    │    - v0: [bestV0±0.1] │
    │    - theta: [bestTheta±0.5°] │
    └──────────┬──────────┘
               │
               ▼
         检查循环次数
               │
         ┌─────┴─────┐
         │  未完成？   │
         └─────┬─────┘
               │ 是
               │
               └──────────┐
                          │
                          ▼
                    返回循环开始
                          │
                          │ 否
                          ▼
┌─────────────────────────────────────┐
│ 最终验证                         │
│ - 使用最优参数模拟轨迹           │
│ - 计算落点误差                 │
│ - 验证误差 < 0.1m              │
└──────────────┬──────────────────┘
               │
               ▼
         返回最优解
```

## 5. 关键算法

### 5.1 RK4积分步骤

```java
private ProjectileState rk4Step(ProjectileState state, ProjectileParameters params) {
    // 计算k1：在当前状态的导数
    double[] k1 = ProjectileDynamics.computeDerivatives(state, params);
    
    // 计算k2：在t + h/2时刻，使用k1预测的状态
    ProjectileState state2 = addDerivative(state, k1, dt * 0.5, params);
    double[] k2 = ProjectileDynamics.computeDerivatives(state2, params);
    
    // 计算k3：在t + h/2时刻，使用k2预测的状态
    ProjectileState state3 = addDerivative(state, k2, dt * 0.5, params);
    double[] k3 = ProjectileDynamics.computeDerivatives(state3, params);
    
    // 计算k4：在t + h时刻，使用k3预测的状态
    ProjectileState state4 = addDerivative(state, k3, dt, params);
    double[] k4 = ProjectileDynamics.computeDerivatives(state4, params);
    
    // 加权平均更新状态
    return new ProjectileState(
        x + (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) * dt / 6,
        y + (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) * dt / 6,
        z + (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) * dt / 6,
        vx + (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) * dt / 6,
        vy + (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]) * dt / 6,
        vz + (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]) * dt / 6,
        time + dt
    );
}
```

### 5.2 二分搜索

#### 仰角搜索（固定初速度）

```java
private BinarySearchResult binarySearchElevation(
    double turretPhi, double targetX, double targetY,
    RobotState robotState, double v0
) {
    double thetaLow = yawModeThetaMin;  // 45度
    double thetaHigh = yawModeThetaMax;  // 65度
    
    for (int i = 0; i < maxBinarySearchIterations; i++) {
        double thetaMid = (thetaLow + thetaHigh) / 2;
        
        // 模拟轨迹
        TrajectorySimulator.TrajectoryResult result = 
            simulator.simulate(turretPhi, thetaMid, robotState, params);
        
        if (!result.reachedTargetHeight) {
            return new BinarySearchResult(false, 0, 0);
        }
        
        // 计算水平距离误差
        double distance = result.getHorizontalDistance();
        double targetDistance = Math.sqrt(targetX*targetX + targetY*targetY);
        double error = distance - targetDistance;
        
        // 调整搜索范围
        if (Math.abs(error) < binarySearchTolerance) {
            return new BinarySearchResult(true, thetaMid, result.flightTime);
        }
        
        if (error > 0) {
            thetaHigh = thetaMid;  // 射程过大，减小仰角
        } else {
            thetaLow = thetaMid;   // 射程过小，增大仰角
        }
    }
    
    return new BinarySearchResult(false, 0, 0);
}
```

#### 初速度搜索（固定仰角）

```java
private BinarySearchVelocityResult binarySearchVelocity(
    double turretPhi, double targetX, double targetY,
    RobotState robotState, double fixedTheta
) {
    double v0Low = v0Min;   // 0 m/s
    double v0High = v0Max;   // 10 m/s
    
    for (int i = 0; i < maxBinarySearchIterations; i++) {
        double v0Mid = (v0Low + v0High) / 2;
        
        // 临时设置初速度
        ProjectileParameters tempParams = params.copy();
        tempParams.v0 = v0Mid;
        
        // 模拟轨迹
        TrajectorySimulator.TrajectoryResult result = 
            simulator.simulate(turretPhi, fixedTheta, robotState, tempParams);
        
        if (!result.reachedTargetHeight) {
            return new BinarySearchVelocityResult(false, 0, 0);
        }
        
        // 计算水平距离误差
        double distance = result.getHorizontalDistance();
        double targetDistance = Math.sqrt(targetX*targetX + targetY*targetY);
        double error = distance - targetDistance;
        
        // 调整搜索范围
        if (Math.abs(error) < binarySearchTolerance) {
            return new BinarySearchVelocityResult(true, v0Mid, result.flightTime);
        }
        
        if (error > 0) {
            v0High = v0Mid;  // 射程过大，减小初速度
        } else {
            v0Low = v0Mid;   // 射程过小，增大初速度
        }
    }
    
    return new BinarySearchVelocityResult(false, 0, 0);
}
```

### 5.3 旋转角修正

#### 修正原理

当小车运动时，需要修正旋转角以补偿小车速度的影响：

```
φ_corrected = groundAngle - correction

correction = (robotVy·cos(groundAngle) - robotVx·sin(groundAngle)) / v_horizontal
```

其中：
- `groundAngle`：目标相对于小车的角度
- `v_horizontal`：抛射体的水平速度
- `robotVx, robotVy`：小车速度分量

#### 物理意义

- 当小车向前移动时，抛射体获得额外的水平速度
- 需要减小旋转角以补偿
- 修正量取决于小车速度和抛射体水平速度的比值

## 6. 性能优化

### 6.1 计算优化

1. **减少RK4步数**：
   - 增加时间步长（0.01秒）
   - 设置最大飞行时间（5.0秒）
   - 限制最大步数（500步）

2. **提前终止**：
   - 旋转角收敛检查（0.5度容差）
   - 二分搜索精度检查（0.001弧度）
   - 最大迭代次数限制

3. **参数缓存**：
   - 避免重复创建对象
   - 复用ProjectileParameters
   - 缓存中间结果

### 6.2 算法优化

1. **智能搜索范围**：
   - 根据初始值设置搜索范围
   - 动态缩小搜索范围
   - 避免无效搜索

2. **并行优化**（可选）：
   - 内层和外层迭代可并行化
   - 多个候选值可同时仿真

3. **近似计算**：
   - 在实时控制中使用近似值
   - 降低精度要求

### 6.3 内存优化

1. **对象复用**：
   - 使用copy()方法避免重复创建
   - 重用ProjectileState对象

2. **避免垃圾回收**：
   - 预分配数组
   - 减少临时对象创建

## 7. 使用建议

### 7.1 参数标定

1. **初速度v₀**：
   - 通过实验测量
   - 考虑发射机构的一致性
   - 建议范围：6-10 m/s

2. **阻力参数kx和升力参数ky**：
   - 使用固定仰角（45度）进行拟合
   - 测量多个距离的落点
   - 使用最小二乘法拟合
   - kx范围：-0.1~0.1
   - ky范围：-0.5~0.5

3. **高度差deltaH**：
   - 精确测量炮口高度
   - 考虑目标高度变化
   - 建议范围：0.5-1.5 m

### 7.2 实时控制

1. **更新频率**：
   - 建议20-50Hz
   - 根据硬件性能调整
   - 避免过度计算

2. **状态管理**：
   - 维护机器人和目标状态
   - 使用低通滤波减少噪声
   - 实现预测机制

3. **容错处理**：
   - 检查求解结果有效性
   - 提供备用方案
   - 记录错误日志

### 7.3 调试技巧

1. **日志输出**：
   - 记录输入参数
   - 输出中间结果
   - 跟踪迭代过程

2. **可视化**：
   - 绘制轨迹曲线
   - 显示误差变化
   - 监控收敛过程

3. **性能分析**：
   - 测量计算时间
   - 识别瓶颈
   - 优化热点代码

### 7.4 比赛应用

1. **自动阶段**：
   - 使用精确参数
   - 预计算常用目标
   - 实现快速响应

2. **手动阶段**：
   - 实时更新目标
   - 适应目标移动
   - 提供辅助瞄准

3. **异常处理**：
   - 处理传感器故障
   - 适应场地变化
   - 保持系统稳定

## 总结

RK4跑打算法通过精确的物理建模和高效的数值计算，为FTC机器人提供了高精度的射击能力。系统架构清晰，模块化设计良好，易于维护和扩展。通过合理配置参数和优化算法，可以在保证精度的同时满足实时性要求。

关键优势：
- **高精度**：RK4积分和双层迭代确保计算精度
- **实时性**：优化后的算法适合实时控制
- **鲁棒性**：多层验证和容错机制
- **灵活性**：支持多种模式和参数配置

适用场景：
- FTC比赛中的目标射击
- 抛射体轨迹预测
- 精确制导系统
- 物理仿真和教学