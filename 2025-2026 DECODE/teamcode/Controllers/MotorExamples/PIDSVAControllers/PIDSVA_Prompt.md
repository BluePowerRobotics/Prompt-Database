
# 任务目标
请根据以下描述的 PID 控制器、SVA 前馈控制器以及组合控制器（PIDSVA）的原理和功能，在 [目标语言/框架] 中实现功能完全等价的类。你需要严格遵循给出的算法细节、方法签名风格（允许适配目标语言的命名规范）以及使用逻辑。

## 背景说明
这些控制器用于机器人运动控制（如 FTC 竞赛中的电机闭环控制）。其中：
- PID 控制器负责根据误差反馈计算修正量。
- SVA 前馈控制器基于静态摩擦、速度、加速度模型提供前馈输出，以提高跟踪精度。
- 组合控制器将 PID 与 SVA 结合，支持多组参数配置（Slot）动态切换，方便在加速、匀速等不同工况下使用不同参数。

---

## 1. PID 控制器（PIDController）

### 原理
标准 PID 控制算法，采用 **误差的比例（P）、积分（I）、微分（D）** 加权和计算输出。包含积分限幅（anti‑windup）和微分项的时间归一化。

### 功能
- 根据目标值（setpoint）和测量值（measurement）以及时间间隔（dt）计算控制输出。
- 支持动态修改 PID 系数。
- 支持重置积分和上次误差（复位状态）。
- 积分限幅可配置（maxI），避免积分饱和。

### 算法公式
```
error = setpoint - measurement
integral += error * dt
对 integral 进行 ±maxI 限幅
derivative = (error - previousError) / dt
output = kP * error + kI * integral + kD * derivative
```
其中 `previousError` 在每次计算后更新为当前的 error。

### 公开方法（需实现）
- 构造函数：
  - `PIDController(kP, kI, kD)` （默认 maxI = 1.0）
  - `PIDController(kP, kI, kD, maxI)`
- `calculate(setpoint, measurement, dt) -> double`
- `reset()`          // integral = 0; previousError = 0
- `setPID(kP, kI, kD)`  // 如果 kI == 0，顺便将 integral 清零
- `setMaxI(maxI)`

---

## 2. SVA 前馈控制器（SVAController）

### 原理
基于 **静态摩擦（Stiction）‑速度（Velocity）‑加速度（Acceleration）** 模型的前馈控制器。它不依赖误差，完全由期望运动状态（速度、加速度）和实际速度计算前馈值。

### 功能
- 仅依赖速度和加速度输入，返回前馈输出。
- 静态摩擦项使用 `sign(velocity)` 提供克服静摩擦的恒定方向力。
- 支持动态修改三个系数。

### 算法公式
```
output = kS * sign(velocity) + kV * velocity + kA * acceleration
```
注：当 velocity = 0 时，`sign(0)` 通常取 0（即不附加静态摩擦项）。

### 公开方法（需实现）
- 构造函数 `SVAController(kS, kV, kA)`
- `calculate(velocity, acceleration) -> double`
- `setSVA(kS, kV, kA)`

---

## 3. 配置类（SlotConfig）

### 用途
用于封装一组 PID + SVA 的全套参数，支持建造者模式的链式设置。该类仅作为数据容器，不包含任何算法逻辑。

### 字段
- `kP` (double) – 比例系数
- `kI` (double) – 积分系数
- `kD` (double) – 微分系数
- `maxI` (double) – 积分上限，默认 1.0
- `kS` (double) – 静态摩擦系数
- `kV` (double) – 速度系数
- `kA` (double) – 加速度系数
- `outputMin` (double) – 输出下限，默认 -1.0
- `outputMax` (double) – 输出上限，默认 1.0

### 链式方法（每个方法返回 `this`）
- `withKP(kP)`
- `withKI(kI)`
- `withKD(kD)`
- `withMaxI(maxI)`
- `withKS(kS)`
- `withKV(kV)`
- `withKA(kA)`
- `withOutputLimits(min, max)`

---

## 4. 组合 PIDSVA 控制器（PIDSVAController）

### 原理
将 PID 反馈与 SVA 前馈相加，输出总控制量。支持多个参数配置（Slot），每个 Slot 包含完整的 `SlotConfig`。可以在运行时切换 Slot，切换时自动重置积分和上一次误差。适用于不同运动阶段（如加速段用高 kP、匀速段用高 kV 前馈）。

### 功能
- 存储多个 `SlotConfig` 对象，通过整数 slot 编号索引。
- 当前激活的 slot 决定使用的 PID 系数、SVA 系数、积分限幅和输出限幅。
- 提供多个重载的 `calculate` 方法：
  - **完整版**：`calculate(setpoint, measurement, velocity, acceleration, dt)`  
    内部计算 `error = setpoint - measurement`，积分、微分，然后 `pid = kP*error + kI*integral + kD*derivative`，  
    `sva = kS * sign(velocity) + kV * velocity + kA * acceleration`，  
    `output = pid + sva`，最后用 `outputMin/outputMax` 裁切后返回。
  - **简化版**：`calculate(setpoint, measurement, dt, VelCycle)`  
    若 `VelCycle == true`：将 setpoint 同时作为速度前馈，加速度前馈为 0（即调用完整版时 `velocity = setpoint, acceleration = 0`）。  
    若 `VelCycle == false`：速度和加速度前馈均为 0（即纯 PID 模式）。
- 支持链式添加 slot 配置：`withSlot(slot, config)` 及 `withSlot0(config)`。
- 支持重置控制器状态（不切换 slot）或重置某个 slot 的配置。
- 切换 slot 时（`setSlot(slot)`）自动调用 `reset()` 清零积分和上次误差。

### 算法细节（完整版计算步骤）
```
error = setpoint - measurement
integral += error * dt
如果 integral > cfg.maxI → integral = cfg.maxI
如果 integral < -cfg.maxI → integral = -cfg.maxI
derivative = (error - previousError) / dt
previousError = error
pid = cfg.kP * error + cfg.kI * integral + cfg.kD * derivative
sva = cfg.kS * sign(velocity) + cfg.kV * velocity + cfg.kA * acceleration
output = pid + sva
如果 output > cfg.outputMax → output = cfg.outputMax
如果 output < cfg.outputMin → output = cfg.outputMin
返回 output
```

### 公开方法（需实现）
- `withSlot0(config)` → 设置 slot 0，返回自身（链式）
- `withSlot(slot, config)` → 设置指定 slot，返回自身
- `setSlot(slot)` → 切换当前 slot，并重置积分/微分状态
- `resetSlot(config)` → 重置 slot 0 的配置（前提是该 slot 已存在）
- `resetSlot(slot, config)` → 重置指定 slot 的配置
- `calculate(setpoint, measurement, dt, VelCycle)` → 上面描述的简化版
- `calculate(setpoint, measurement, velocity, acceleration, dt)` → 完整版
- `reset()` → 仅重置积分和 previousError（不切换 slot）

### 使用示例（期望的调用方式）
```java
SlotConfig cfg = new SlotConfig()
    .withKP(1.2).withKI(0.1).withKD(0.05).withMaxI(0.5)
    .withKS(0.2).withKV(0.8).withKA(0.1)
    .withOutputLimits(-1.0, 1.0);

PIDSVAController controller = new PIDSVAController()
    .withSlot0(cfg)
    .withSlot(1, anotherCfg);

controller.setSlot(0);
double output = controller.calculate(targetPos, currentPos, currentVel, currentAcc, dt);
```

---

## 实现要求
- 请为目标语言编写三个类（或结构体）：`PIDController`，`SVAController`，`SlotConfig`，`PIDSVAController`。
- 保持所有逻辑与原描述完全一致，尤其是积分限幅、微分处理、输出限幅和 sign(velocity) 的行为（速度为零时不添加 kS 项）。
- 注意处理 dt 可能极小或为零的边界情况（建议 if dt <= 0 则直接返回上次输出或 0，可自由设计简单防护）。
- 如果目标语言不支持方法重载，请用不同方法名区分（例如 `calculateFull` 和 `calculateSimple`）。
- 代码应包含必要的注释，解释关键步骤。
- 若有必要，可额外提供一个简短的使用示例。

