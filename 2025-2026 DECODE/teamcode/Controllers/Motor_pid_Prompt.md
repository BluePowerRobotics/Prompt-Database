## 单电机 PID 速度控制器实现规范 (LLM Prompt)

你是一位 FTC 机器人控制软件工程师。请实现一个**单电机 PID 速度控制器**，用于精确控制直流电机的转速（如飞轮、发射机构、升降机构等），并具备双 PID 参数切换、目标速度判稳、功率限幅和自动归零等特性。

---

### 1. 功能概述

- 对单个带编码器的直流电机（`DcMotorEx`）进行**速度闭环控制**。
- 支持**两套 PID 参数**，根据目标速度自动切换（如低速和高速用不同增益）。
- 提供**目标速度判稳**，以速度误差容差判断电机是否达到设定速度。
- 包含**功率限幅**（最小功率防止堵转不转，最大功率保护电机）。
- 当目标速度为 0 时自动停止电机并重置 PID 积分器。
- 暴露当前功率、速度、编码器位置等反馈信息，便于调试。

---

### 2. 硬件参数与配置

- **电机类型**：`DcMotorEx`（需 encoder 支持 `getVelocity()`）。
- **运行模式**：`RUN_USING_ENCODER`（PID 计算时使用）。
- **零功率行为**：设置为 `BRAKE`，防止自由滑行。
- **方向配置**：构造函数中根据 `ifReverse` 参数设置 `FORWARD` 或 `REVERSE`。
- **初始化流程**：
  1. 从 `HardwareMap` 获取电机实例。
  2. 调用 `STOP_AND_RESET_ENCODER` 重置编码器。
  3. 设置运行模式为 `RUN_WITHOUT_ENCODER`（初始状态，PID 执行时会切换）。
  4. 配置制动和方向。

---

### 3. 双 PID 参数切换

- **低速 PID 参数集**（`PIDSwitchSpeed` 以下）：
  - `k_p_1`, `k_i_1`, `k_d_1`, `max_i_1`, `MinPower_1`
- **高速 PID 参数集**（`PIDSwitchSpeed` 及以上）：
  - `k_p_2`, `k_i_2`, `k_d_2`, `max_i_2`, `MinPower_2`
- 切换阈值 `PIDSwitchSpeed` 可动态配置。
- `setTargetSpeed()` 方法中，根据 `targetSpeed` 与阈值的比较，调用 `pidController.setPID()` 更新 PID 增益，并调用 `setMaxI()` 更新积分限幅。

---

### 4. 核心控制方法：`boolean setTargetSpeed(int targetSpeed)`

#### 4.1 特殊处理：目标速度为 0
- 设置电机功率为 0。
- 重置 PID 控制器（清空积分和微分历史）。
- 记录当前时间到 `previous_time`（避免后续计算 dt 异常）。
- 立即返回 `true`。

#### 4.2 模式准备
- 切换电机模式为 `RUN_USING_ENCODER`，以确保速度反馈可用。

#### 4.3 时间步长与速度测量
- 获取当前系统时间 `current_time`（毫秒）。
- 计算时间增量 `dt = current_time - previous_time`（单位毫秒，传递给 PID 计算）。
- 如果 `dt <= 0`，设为 1 防止除零。
- 通过 `getVelocity(AngleUnit.DEGREES)` 获取当前电机速度（单位：度/秒）作为反馈值。
- 更新 `previous_time`。

#### 4.4 PID 计算
- 调用 `pidController.calculate(targetSpeed, current_speed, dt)` 得到输出功率 `Power`。

#### 4.5 功率限幅
- 若 `targetSpeed > 0` 且低于 `PIDSwitchSpeed`，使用 `MinPower_1` 作为下限，上限为 1.0。
- 若 `targetSpeed > 0` 且不低于 `PIDSwitchSpeed`，使用 `MinPower_2` 作为下限，上限为 1.0。
- 限幅采用 `Range.clip()` 或等价方法，防止电机因输出过小而停转，并防止负值出现（因只处理正转场景）。
- 将限幅后的功率设置给电机。

#### 4.6 判稳
- 计算 `|targetSpeed - current_speed|` 是否小于 `SpeedTolerance`（可配置全局阈值）。
- 若小于容差，返回 `true`；否则返回 `false`。

---

### 5. 配套 PID 控制器

- 使用一个独立的 `PIDController` 类，支持：
  - `setPID(kP, kI, kD)` 动态更新增益。
  - `setMaxI(maxIntegral)` 限制积分项幅度，防止积分饱合。
  - `calculate(setpoint, processVariable, dt)` 返回计算结果（-1 到 1 范围或自定义）。
  - `reset()` 清除积分累积和上一次误差。
- PID 公式建议采用**位置式**或**增量式**，并处理好微分冲击（可选低通滤波）。

---

### 6. 公共配置方法

#### 构造参数
- `HardwareMap hardwareMap`
- `String motorName` – 电机在硬件配置中的名字。
- `boolean ifReverse` – 是否反转电机方向。

#### 批量参数设置
- `void PIDsetting(int PIDSwitchSpeed, double MinPower_1, double k_p_1, double k_i_1, double k_d_1, double max_i_1, double MinPower_2, double k_p_2, double k_d_2, double k_i_2, double max_i_2)`
- 该方法将以上所有 PID 相关参数一次性赋值给成员变量，便于 Dashboard 调参。

---

### 7. 反馈接口（Get 方法）

- `double getPower()` – 返回当前电机功率。
- `double getCurrent_speed()` – 返回当前测量速度（度/秒）。
- `double getCurrent_encoder()` – 返回当前编码器值（可用于位置监控）。

---

### 8. 使用示例

```java
// 初始化电机控制器，名称为 "shooter"，反转方向
Motor_pid shooter = new Motor_pid(hardwareMap, "shooter", true);

// 在 OpMode 循环中设置目标速度
boolean atSpeed = shooter.setTargetSpeed(1500); // 目标 1500 deg/s
if (atSpeed) {
    // 可以发射
}

// 停机
shooter.setTargetSpeed(0);
```

---

### 9. 代码设计注意事项

- 所有速度单位保持一致（如度/秒），可通过 `AngleUnit` 枚举切换。
- 电机模式切换应尽量平滑，避免频繁 `STOP_AND_RESET_ENCODER`。
- PID 参数的限幅逻辑需确保在目标速度正转时功率不会被限制到负值，以免引起抖动。
- 判稳容差 `SpeedTolerance` 可根据实际飞轮惯性调整。
- 使用 `@Config` 注解支持 FTC Dashboard 在线调参。

---

请根据以上规范，实现一个 Java 类 `Motor_pid`（或等效命名），确保与 FTC 硬件完全兼容，并提供清晰的注释和文档。