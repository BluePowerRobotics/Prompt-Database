# TurretDegreeController 炮台旋转控制器

## 概述

`TurretDegreeController` 类用于控制 FTC 比赛中的炮台（Turret）双轴旋转机构，实现高精度的位置闭环控制。

## 硬件组成

| 组件 | 类型 | 功能 |
|------|------|------|
| rollMotor | DcMotorEx | 水平旋转电机（绕 Z 轴），使用 PIDSVA 闭环控制 |
| yawServo | Servo | 仰角舵机（绕 Y 轴），直接位置控制 |

## 核心控制算法

### rollMotor - PIDSVA 位置闭环

参考 `VoltageOutMotor_Degree.java` 实现，使用 PIDSVA（比例-积分-微分-静摩擦-速度-加速度）前馈控制器：

```
输出电压 = kP × 误差 + kI × 积分 + kD × 微分 + kS × sign(速度) + kV × 速度 + kA × 加速度
```

- **P (kP)**: 比例系数，用于消除当前误差
- **I (kI)**: 积分系数，用于消除稳态误差
- **D (kD)**: 微分系数，用于抑制震荡
- **S (kS)**: 静摩擦补偿
- **V (kV)**: 速度前馈
- **A (kA)**: 加速度前馈

### yawServo - 舵机位置控制

舵机本身是位置执行器，无需额外闭环控制。角度与舵机位置的映射关系：

```
servoPosition = (targetYaw - YAW_ANGLE_MIN) / (YAW_ANGLE_MAX - YAW_ANGLE_MIN)
```

## 主要方法

### 构造函数

```java
// 完整构造函数（指定电机名称）
TurretDegreeController(HardwareMap hardwareMap, String rollMotorName, String yawServoName, Telemetry telemetry)

// 快速构造函数（使用默认名称 "rollMotor", "yawServo"）
TurretDegreeController(HardwareMap hardwareMap, Telemetry telemetry)

// 旧版构造函数（仅 hardwareMap）
TurretDegreeController(HardwareMap hardwareMap)
```

### 控制方法

| 方法 | 说明 |
|------|------|
| `rotateTo(roll, yaw)` | 旋转到指定角度，返回是否到达目标 |
| `rotateRollTo(targetAngle)` | roll 轴 PIDSVA 闭环控制 |
| `rotateYawTo(targetAngle)` | yaw 轴舵机控制 |
| `update()` | 每帧调用的总更新函数（推荐在主循环使用） |

### 设置方法

| 方法 | 说明 |
|------|------|
| `setTargetRoll(roll)` | 设置目标水平角度 |
| `setTargetYaw(yaw)` | 设置目标仰角 |
| `setTarget(roll, yaw)` | 同时设置两个目标 |
| `resetAngles()` | 重置当前位置为零点 |

### 状态查询方法

| 方法 | 说明 |
|------|------|
| `reachedTarget()` | 检查是否到达目标 |
| `get_angle()` | 获取当前 [roll, yaw] 角度 |
| `getTarget()` | 获取目标 [roll, yaw] 角度 |
| `getAimError()` | 获取瞄准误差 |
| `stop()` | 停止电机 |

## 使用示例

### 方式一：使用 update() 总函数（推荐）

```java
// 初始化
turret = new TurretDegreeController(hardwareMap, telemetry);

// 主循环
public void loop() {
    turret.setTarget(45.0, 90.0);  // 设置目标
    turret.update();                 // 每帧调用

    if (turret.reachedTarget()) {
        // 已到达目标，可以进行射击等操作
    }
}
```

### 方式二：使用 rotateTo()

```java
// 初始化
turret = new TurretDegreeController(hardwareMap, telemetry);

// 主循环
public void loop() {
    if (turret.rotateTo(45.0, 90.0)) {
        // 已到达目标
    }
}
```

## Dashboard 热调参

以下参数可在 FTC Dashboard 中实时调整：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| kP | 比例系数 | 1.0 |
| kI | 积分系数 | 0.0 |
| kD | 微分系数 | 0.0 |
| maxI | 积分上限 | 1.0 |
| kS | 静摩擦系数 | 0.0 |
| kV | 速度系数 | 0.0 |
| kA | 加速度系数 | 0.0 |
| outputMin | 输出电压最小值 | -14.0 |
| outputMax | 输出电压最大值 | 14.0 |

## 角度范围

| 轴 | 最小值 | 最大值 |
|----|--------|--------|
| Roll | -180° | 180°（自动归一化） |
| Yaw | 0° | 180° |

## 注意事项

1. **yawServo 是 Servo 类型**：不能使用 `getCurrentPosition()`（那是 DcMotor 的方法），应使用 `getPosition()` 获取当前位置
2. **RUN_WITHOUT_ENCODER 模式**：rollMotor 使用此模式，编码器仅用于位置反馈
3. **电压补偿**：通过 `VoltageOut` 类根据电池电压动态调整输出功率
4. **时间间隔 (dt)**：在 `update()` 和 `rotateRollTo()` 中自动计算，用于微分项和积分项