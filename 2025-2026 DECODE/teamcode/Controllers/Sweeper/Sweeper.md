# Sweeper 控制器说明

## 概述
`Sweeper` 是 FTC 机器人中用于控制清扫器（Sweeper）电机的驱动类。清扫器通常用于收集、传递或释放场地上的得分物（如“工件”）。该类通过**速度闭环**（利用电机内置 PID）的方式精确控制电机转速，实现“吃入”、“给出工件”、“反向输出”和“停止”四种工作模式。

> 代码中的 TODO 提示未来可能改用纯粹的电压开环或更高级的速度闭环算法。当前版本直接使用 `DcMotorEx.setVelocity()`，底层由电机控制器完成速度闭环。

## 硬件映射
- 电机名称：`"sweeperMotor"`
- 电机类型：`DcMotorEx`（支持编码器速度控制）
- 零功率行为：`FLOAT`（断电后惯性滑行，不刹车）

## 关键参数

| 静态变量 | 默认值 | 单位 | 说明 |
|----------|--------|------|------|
| `EatVel` | 1960 | ticks/s | “吃入”模式的目标速度（正转，收集物品） |
| `GiveTheArtifactVel` | 1960 | ticks/s | “给出工件”模式的目标速度（正转，送出工件） |
| `OutputVel` | -960 | ticks/s | “反向输出”模式的目标速度（反转，吐出物品） |
| `ForR` | 0 | - | 电机方向选通：`0` 为 `REVERSE`，`1` 为 `FORWARD` |

所有静态变量均被 `@Config` 注解，可在 FTC Dashboard 中实时调整，便于调试。

## 控制流程与算法
`Sweeper` 的本质是一个**目标速度设定器**。
- 外部调用 `setEat()` / `setGiveArtifact()` / `setOutput()` / `setStop()` 修改内部的 `targetVelocity` 变量。
- 主循环调用 `update()` 时，将 `targetVelocity` 写入电机：`motor.setVelocity(targetVelocity)`。
- 电机自身的速度 PID 控制器会自动调节 PWM 输出，使实际转速趋近目标值，实现速度闭环。

### 方向控制逻辑
- 构造时根据 `ForR` 的值调用 `setDirection()`：
  - `ForR == 0` → 电机方向设为 `REVERSE`
  - `ForR == 1` → 电机方向设为 `FORWARD`
- 该设计用于适配不同的机械安装方向，只需修改 `ForR` 即可统一“正转＝吃入”的语义。

### 各模式速度定义
- **吃入**（`EatVel`）：正转 1960 ticks/s，通常用于将场上物品卷入机器人内部。
- **给出工件**（`GiveTheArtifactVel`）：正转 1960 ticks/s，速度大小与吃入相同，由机械设计决定方向（如通过单向轮或倒转机构送出）。
- **反向输出**（`OutputVel`）：负值 -960 ticks/s，反转较慢，常用于谨慎地将物品从收集器中排出。
- **停止**：目标速度为 `0`。

## 主要方法

| 方法 | 说明 |
|------|------|
| `setEat()` | 将目标速度设为 `EatVel`（收集） |
| `setGiveArtifact()` | 将目标速度设为 `GiveTheArtifactVel`（送出工件） |
| `setOutput()` | 将目标速度设为 `OutputVel`（反向输出） |
| `setStop()` | 将目标速度设为 `0`（停止） |
| `setPower(double power)` | 直接设置电机功率（开环控制，用于测试或特殊场景） |
| `update()` | **必须在每个循环周期调用**，将当前目标速度写入电机 |
| `getVel()` | 返回电机实际速度（ticks/s） |
| `getPower()` | 返回电机当前功率（-1 到 1） |
| `getCurrent()` | 返回电机当前电流（安培） |
| `setTelemetry()` | 将速度、功率、电流数据发送到 Driver Station 遥测 |

## 典型使用示例
```java
// 初始化
Sweeper sweeper = new Sweeper(hardwareMap, telemetry);

// 循环中
while (opModeIsActive()) {
    if (gamepad1.a) {
        sweeper.setEat();
    } else if (gamepad1.b) {
        sweeper.setGiveArtifact();
    } else if (gamepad1.x) {
        sweeper.setOutput();
    } else {
        sweeper.setStop();
    }
    
    sweeper.update();
    sweeper.setTelemetry();
}
```

## 注意事项
1. **电机编码器线缆**必须正确连接，否则 `setVelocity` 将因无反馈而报错或失控。
2. 目标速度单位取决于电机编码器的分辨率（通常是 **ticks per second**，REV 电机为 28 个脉冲/转，乘上减速比）。
3. 使用速度闭环时，电池电压变化会自动被 PID 补偿，但若机械负载过大可能导致电流过高，建议通过 `getCurrent()` 监控异常电流。
4. 若需更改电机方向，请修改 `ForR`（0 或 1），并确保重新构造对象或调用 `setDirection()`（当前该方法是私有的，仅在构造时调用；若需运行时切换，可将其改为公开）。