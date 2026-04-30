## IMU 传感器封装实现规范 (LLM Prompt)

你是一位 FTC 机器人软件工程师。请实现一个 **IMU 传感器封装类**，用于在 FTC 项目中可靠地获取机器人朝向角度，并包含初始化重试、异常处理、复位航向等功能。该类应基于 REV Hub 内置的 IMU 或兼容的通用 IMU 接口。

---

### 1. 类职责与概述

- 封装 `com.qualcomm.robotcore.hardware.IMU` 接口，提供更鲁棒的初始化和角度读取。
- 支持自定义 `RevHubOrientationOnRobot`（即 IMU 在机器人上的安装方向），以便正确解算偏航角。
- 包含初始化失败后的自动重试机制。
- 提供获取当前偏航角（Yaw）、俯仰角（Pitch）、横滚角（Roll）的统一方法。
- 提供重置偏航角（设为 0）的功能。

---

### 2. 核心数据结构

#### 2.1 成员变量
- `IMU imu` – 底层 IMU 硬件对象。
- `YawPitchRollAngles yawPitchRollAngles` – 存储最近一次读取的欧拉角。
- `boolean isInited` – IMU 是否成功初始化。
- `IMU.Parameters parameters` – IMU 参数对象，包含 `RevHubOrientationOnRobot`。

#### 2.2 构造参数
- `HardwareMap hardwareMap` – FTC 硬件映射。
- `String deviceName` – 配置文件中 IMU 的名称（如 `"imu"`）。
- `RevHubOrientationOnRobot orientation` – IMU 物理安装方向（由 FTC 文档中的方向枚举定义）。

---

### 3. 初始化流程

构造函数中完成以下步骤：
1. 从 `hardwareMap` 获取 `IMU` 实例（`hardwareMap.get(IMU.class, deviceName)`）。
2. 创建 `IMU.Parameters` 对象，传入 `orientation`。
3. 调用 `imu.initialize(parameters)` 初始化 IMU，并将结果赋值给 `isInited`。
4. **如果初始化成功**，立即尝试读取一次角度（`imu.getRobotYawPitchRollAngles()`），并用 `try-catch` 捕获可能的异常。若出现异常，使用 `RobotLog.addGlobalWarningMessage` 记录警告。
5. **如果初始化失败**（`isInited == false`），再次调用 `imu.initialize(parameters)` 进行重试，并同样尝试读取一次角度。

---

### 4. 角度读取方法

#### `YawPitchRollAngles getYawPitchRollAngles()`
- **前置条件检查**：如果 `isInited` 为 `true`，尝试调用 `imu.getRobotYawPitchRollAngles()` 获取最新角度。
  - 使用 `try-catch` 捕获异常，若捕获到，记录全局警告，但仍返回之前存储的 `yawPitchRollAngles`（或 `null` 由外部判断）。
- **如果尚未初始化**（`isInited == false`）：尝试重新调用 `imu.initialize(parameters)`，然后尝试读取角度（同样用 `try-catch` 处理异常）。
- 返回值：存储的 `yawPitchRollAngles`，包括 Yaw、Pitch、Roll 信息。

#### `double getYaw(AngleUnit angleUnit)`
- 便捷方法，直接返回 `getYawPitchRollAngles().getYaw(angleUnit)`。
- 单位由 `AngleUnit` 枚举指定（如 `RADIANS` 或 `DEGREES`）。

---

### 5. 偏航角重置

#### `YawPitchRollAngles reset()`
- 检查 `isInited`，若已初始化则调用 `imu.resetYaw()` 将当前偏航角归零。
- 随后立即读取并返回新的 `YawPitchRollAngles`。
- 若未初始化，执行与读取相同的重试逻辑：重新 `initialize`，然后尝试读取角度。

---

### 6. 初始化状态查询

#### `boolean ifInitiated()`
- 返回 `isInited`，便于外部判断 IMU 是否正常工作。

---

### 7. 异常处理与鲁棒性设计

- 所有与 IMU 的硬件交互必须包裹在 `try-catch` 中，避免因瞬时故障导致程序崩溃。
- 异常通过 `RobotLog.addGlobalWarningMessage` 记录，便于调试。
- 在构造函数和 `getYawPitchRollAngles()` 中，若出现异常，不抛出，而是保留上一次的有效数据（或初始 null）。
- 提供自动重试初始化机制：当检测到 `isInited` 为 `false` 时，每次调用读取方法都会尝试重新初始化。

---

### 8. 使用示例

```java
// 创建 IMUSensor，假设 IMU 的 USB 方向为 LOGO_FACING_UP, USB_FACING_FORWARD
IMUSensor imuSensor = new IMUSensor(hardwareMap, "imu",
        new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

// 在 OpMode 循环中获取偏航角（弧度）
double yawRad = imuSensor.getYaw(AngleUnit.RADIANS);

// 重置偏航角（将当前方向设为 0）
imuSensor.reset();
```

---

### 9. 代码组织要求

- 类名：`IMUSensor`（或 `ImuWrapper`）。
- 包路径建议：`org.firstinspires.ftc.teamcode.Controllers`。
- 所有方法应为**非静态**，成员变量为 `private`。
- 确保 `YawPitchRollAngles` 的获取不产生不必要的对象创建，但可直接返回 IMU 内部对象（线程安全由调用方保证）。

---

请根据以上规范，生成完整的 Java 类实现，并添加详尽的注释。实现时注意遵循 FTC 官方推荐的 IMU 使用方式，并保证在 IMU 硬件出现临时故障时系统具有足够的容错性。