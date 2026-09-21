# RobotPosition 使用指南

## 概述

`RobotPosition` 是 FTC 机器人代码库中的位置管理类，基于 Road Runner 框架实现实时的机器人位姿追踪。该类采用单例模式设计，提供统一的机器人位置与速度查询接口。

## 初始化

在使用前，必须先调用初始化方法：

```java
// 方式一：指定初始位姿 (x=0, y=0, heading=0)
RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose);

// 方式二：不指定初始位姿（默认为原点）
RobotPosition RobotPositioninit(HardwareMap hardwareMap, null);
```

**示例：**

```java
// 在 OpMode 初始化阶段
RobotPosition.getInstance().RobotPositioninit(hardwareMap, new Pose2d(0, 0, 0));
```

## 获取实例

```java
RobotPosition position = RobotPosition.getInstance();
```

> **注意：** 在调用 `getInstance()` 之前，必须确保已调用过 `RobotPositioninit()` 进行初始化，否则会抛出 `IllegalStateException`。

## 核心方法

### 位置更新（每帧调用）

```java
Pose2d update()
```

在 `loop()` 循环中调用此方法，更新机器人当前位置信息：

```java
@Override
public void loop() {
    RobotPosition.getInstance().update();
    // 后续可获取位置信息
}
```

### 位置信息获取

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `getPose2d()` | `Pose2d` | 获取完整位姿对象 |
| `getX()` | `double` | 获取 X 坐标（英寸） |
| `getY()` | `double` | 获取 Y 坐标（英寸） |
| `getTheta()` | `double` | 获取航向角（弧度） |

### 速度信息获取

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `getVx()` | `double` | X 轴线速度（英寸/秒） |
| `getVy()` | `double` | Y 轴线速度（英寸/秒） |
| `getOmega()` | `double` | 角速度（弧度/秒） |

### IMU 接口（航向角）

IMU 复用 Road Runner 已初始化的实例（设备名 `"imu"`，方向在 `MecanumDrive.PARAMS` 中配置）。

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `getYawPitchRollAngles()` | `YawPitchRollAngles` | 获取 IMU 的 yaw/pitch/roll 角 |
| `getYaw(AngleUnit)` | `double` | 获取指定单位的 yaw（航向角） |
| `resetYaw()` | `void` | 重置 IMU yaw 为 0 |

> **注意：** `resetYaw()` 会清零 IMU yaw。由于 Road Runner 定位依赖 IMU yaw 计算航向增量，运行中调用会破坏位姿估计，仅应在初始化/标定时使用。

## 使用示例

### 基本定位查询

```java
// 在 TeleOp 或 Autonomous 的 loop() 中
public void loop() {
    RobotPosition.getInstance().update();

    double x = RobotPosition.getInstance().getX();
    double y = RobotPosition.getInstance().getY();
    double theta = RobotPosition.getInstance().getTheta();

    telemetry.addData("X", x);
    telemetry.addData("Y", y);
    telemetry.addData("Heading", theta);
    telemetry.update();
}
```

### 结合 Road Runner 动作使用

```java
// 获取当前位姿用于路径规划
Pose2d currentPose = RobotPosition.getInstance().getPose2d();

// 跟随轨迹
RoadRunnerTrajectory trajectory = drive.trajectoryBuilder(currentPose)
    .splineTo(new Vector2d(24, 24), Math.PI / 2)
    .build();
drive.followTrajectory(trajectory);
```

## 依赖说明

- **Road Runner**：使用 `MecanumDrive` 和 `Localizer` 进行位姿估计
- **硬件**：依赖 `HardwareMap` 进行传感器和电机配置

## 注意事项

1. **初始化时机**：在 OpMode 的 `init()` 阶段完成初始化
2. **更新频率**：必须在主循环中每帧调用 `update()` 以确保数据实时
3. **坐标系**：Road Runner 默认使用英寸为单位，X 轴指向前进方向，Y 轴指向左侧
4. **航向角**：使用弧度制，逆时针为正方向
