## 通用电机 PID 核心控制器实现规范 (LLM Prompt)

你是一位 FTC 机器人控制软件工程师。请实现一个**通用电机 PID 核心控制器 `MotorPIDCore`**，用于单个带编码器的直流电机的**速度闭环控制**和**角度位置控制**。该控制器应支持双 PID 参数切换、速度判稳、限幅保护，并提供角度转向功能，可作为其他子系统（如飞轮、升降机构、炮台旋转）的基础组件。

---

### 1. 功能概述

- 对单个 `DcMotorEx` 电机实现速度 PID 闭环。
- 支持两套 PID 参数，根据目标速度自动切换（低速/高速档）。
- 提供**角度转向模式**：以位置 PID 控制电机转动到目标角度（编码器计数值）。
- 具备功率限幅（最小功率、最大 ±1.0）、判稳阈值等保护机制。
- 暴露电机功率、速度、编码器位置等反馈，便于遥测。
- 提供两个构造函数：基础版本（无遥测）和带 `Telemetry` 的版本。

---

### 2. 硬件配置与初始化

- **电机类型**：`DcMotorEx`，需支持 `getVelocity()` 和 `getCurrentPosition()`。
- **运行模式**：默认 `RUN_WITHOUT_ENCODER`；速度控制时切换为 `RUN_USING_ENCODER`；角度控制时使用 `RUN_WITHOUT_ENCODER`（以原始功率控制位置）。
- **零功率行为**：`BRAKE`。
- **方向配置**：构造函数参数 `ifReverse` 控制正反转。
- **编码器初始化**：构造时调用 `STOP_AND_RESET_ENCODER` 重置计数。

---

### 3. 双 PID 参数与切换

- 维护两套 PID 参数集合：
  - **低速参数** (`PIDSwitchSpeed` 以下)：`MinPower_1`, `k_p_1`, `k_i_1`, `k_d_1`, `max_i_1`
  - **高速参数** (`PIDSwitchSpeed` 及以上)：`MinPower_2`, `k_p_2`, `k_i_2`, `k_d_2`, `max_i_2`
- 共用 PID 控制器实例 `PIDController`，在 `setTargetSpeed()` 中根据目标速度更新增益。
- 提供通用方法 `setPID(double kP, double kI, double kD, double maxI)` 便于外部调参。

---

### 4. 核心控制方法

#### 4.1 `boolean setTargetSpeed(int targetSpeed)` – 速度 PID 控制

**特殊处理**：若 `targetSpeed == 0`，停转电机，重置 PID，并立即返回 `true`。

**过程**：
1. 根据 `targetSpeed` 与 `PIDSwitchSpeed` 比较，设置对应的 PID 增益和积分上限。
2. 切换电机模式为 `RUN_USING_ENCODER`。
3. 读取当前时间（毫秒）、编码器位置和用 `getVelocity(AngleUnit.DEGREES)` 测量的当前速度。
4. 计算时间步长 `dt = current_time - previous_time`，最小设为 1 ms。
5. 调用 `pidController.calculate(targetSpeed, current_speed, dt)` 得到输出功率 `Power`。
6. 功率限幅：若目标速度 > 0，根据所使用的参数集取 `MinPower_1` 或 `MinPower_2` 作为下限，上限为 1.0。使用 `Range.clip()` 限制。
7. 设置电机功率，更新 `previous_time`。
8. 返回 `true` 如果 `|targetSpeed - current_speed| < SpeedTolerance`，否则 `false`。

#### 4.2 `boolean turn(double targetAngle, double currentAngle)` – 角度位置 PID 控制

- 记录目标角度和开始转向标志 `isTurning = true`。
- 如果目标角度为 NaN 或近似于 0（如 < 0.0001），停止电机并重置 PID，返回 `true`。
- 切换电机模式为 `RUN_WITHOUT_ENCODER`（直接功率控制，位置闭环由外部提供反馈）。
- 计算角度误差 `angleError = targetAngle - currentAngle`。
- 计算时间步长 `dt`，使用 `pidController.calculate(0, angleError, dt)` 产生功率（设定值为 0，误差为 `angleError`）。这将使控制器将误差驱动到 0。
- 功率限幅到 `[-1, 1]`。
- 设置电机功率。
- 返回 `true` 如果 `|angleError| < 1`（度误差阈值）。

#### 4.3 `void stopTurn()`
- 停止电机，重置 PID，清除 `isTurning` 标志。

---

### 5. 反馈接口

- `double getVelocity()` – 电机编码器速度（原始值，单位取决于电机内部配置）。
- `double getCurrent_encoder()` – 最近记录的编码器位置。
- `double getPower()` – 当前电机功率。
- `double getCurrent_speed()` – 最近测量的速度（度/秒）。
- `double getTargetAngle()` – 当前转向的目标角度。
- `boolean isTurning()` – 是否正在执行角度转向。
- `boolean isAtTarget()` – 判断 `|targetAngle - currentPosition * degreePertick| < 1`，需提供转换因子 `degreePertick`。

---

### 6. 构造函数设计

#### 基础版
```java
public MotorPIDCore(HardwareMap hardwareMap, String motorName, boolean ifReverse)
```
- 初始化电机，使用默认低速 PID 参数。

#### 遥测版
```java
public MotorPIDCore(HardwareMap hardwareMap, Telemetry telemetry, String motorName, boolean ifReverse)
```
- 额外存储 `telemetry` 以便调试输出。
- 使用通用 PID 参数（单套，但可后续调用 `setPID` 或通过 `setTargetSpeed` 内部切换）。

---

### 7. 使用示例

```java
// 初始化电机（带遥测）
MotorPIDCore armMotor = new MotorPIDCore(hardwareMap, telemetry, "arm_motor", false);

// 设置 PID 增益
armMotor.setPID(0.05, 0.01, 0.001, 1.0);

// 速度控制模式：以 1200 deg/s 旋转
boolean atSpeed = armMotor.setTargetSpeed(1200);

// 角度转向模式：转到 90 度位置（需要提供角度反馈 currentAngle）
boolean reached = armMotor.turn(90.0, currentAngle);
if (reached) armMotor.stopTurn();

// 查询状态
if (armMotor.isTurning()) {
    telemetry.addData("Target", armMotor.getTargetAngle());
}
```

---

### 8. 设计要点与注意事项

- `degreePertick` 应由外部设置，用于将编码器计数转换为角度（例如 `degreePertick = 360.0 / TICKS_PER_REV`），否则 `isAtTarget()` 无法正确判断。
- 角度控制的 `turn` 方法依赖于外部提供的角度反馈（不读取编码器角度），因此调用方需要维护真实角度；内部仅做 PID 计算。
- PID 控制器需支持 `calculate(setpoint, processVariable, dt)` 以及 `reset()`、`setMaxI()` 方法。
- 时间 `dt` 单位为毫秒，但可适配秒（注意乘法因子）。若使用秒，需调整 PID 增益。
- 速度判稳容差 `SpeedTolerance` 可设为静态变量便于全局调整。
- 所有硬件的读写应避免异常导致崩溃，必要时捕获并记录。
- 该类可作为基础组件，派生更专业的控制器（如飞轮、炮台），因此需暴露足够的 protected/public 字段。

---

请根据上述规范，实现完整的 `MotorPIDCore` 类。代码应包含清晰的 Javadoc 注释，并处理好模式切换和异常情况。