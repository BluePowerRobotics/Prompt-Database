# PID 控制器使用指南

## 概述

`PID` 目录包含FTC机器人控制所需的PID控制器系列类，支持比例-积分-微分控制、SVA前馈控制以及多slot配置。

---

## 类说明

### 1. PIDController

基础PID控制器，用于根据目标值和测量值计算控制输出。

```java
// 创建PID控制器
PIDController pid = new PIDController(kP, kI, kD);
PIDController pid = new PIDController(kP, kI, kD, maxI, iZone); // 带积分上限与Izone

// 每帧调用
double output = pid.calculate(setpoint, measurement, dt);

// 重置积分和微分状态
pid.reset();

// 运行时修改参数
pid.setPID(newKP, newKI, newKD);
pid.setMaxI(newMaxI);
pid.setIZone(newIZone);
double currentIZone = pid.getIZone();
```

**参数说明：**
- `kP`: 比例系数，增大响应速度
- `kI`: 积分系数，消除稳态误差
- `kD`: 微分系数，抑制超调
- `maxI`: 积分上限，防止积分饱和
- `iZone`: 误差进入该范围后才累加积分，超出则清零积分
- `dt`: 时间间隔（秒），两次调用之间的时间差

### 2. SVAController

SVA（静态摩擦、速度、加速度）前馈控制器，用于计算电机控制的前馈项。

```java
// 创建SVA控制器
SVAController sva = new SVAController(kS, kV, kA);

// 计算前馈输出
double feedforward = sva.calculate(velocity, acceleration);

// 运行时修改参数
sva.setSVA(newKS, newKV, newKA);
```

**参数说明：**
- `kS`: 静态摩擦系数
- `kV`: 速度系数
- `kA`: 加速度系数

### 3. PIDSVAController

结合PID和SVA前馈的高级控制器，支持多slot配置，可根据不同场景切换参数。

```java
// 创建PIDSVA控制器，配置slot0
SlotConfig slot0 = new SlotConfig()
    .withKP(0.1).withKI(0.01).withKD(0.001)
    .withMaxI(0.5).withIZone(10.0)
    .withKS(0.05).withKV(0.12).withKA(0.01)
    .withOutputLimits(-1.0, 1.0);

PIDSVAController controller = new PIDSVAController().withSlot0(slot0);

// 配置多个slot
SlotConfig slot1 = new SlotConfig()
    .withKP(0.2).withKI(0.0).withKD(0.002)
    .withKS(0.03).withKV(0.10).withKA(0.005)
    .withOutputLimits(-0.8, 0.8);
controller.withSlot(1, slot1);

// 切换slot
controller.setSlot(1);

// 运行中更新某个slot的参数（需先构建新SlotConfig再resetSlot）
controller.resetSlot(new SlotConfig().withKP(0.3));          // 更新0号slot
controller.resetSlot(1, new SlotConfig().withKP(0.25));     // 更新1号slot

// 位置闭环（参考速度/加速度由控制器内部按 setpoint 变化率推导）
double output = controller.calculate(setpoint, measurement, dt, false);

// 速度闭环（setpoint 即参考速度）
double output = controller.calculate(setpoint, measurement, dt, true);

// 重置状态
controller.reset();
```

### 4. SlotConfig

建造者模式配置类，用于设置PID和SVA参数。

```java
SlotConfig config = new SlotConfig()
    .withKP(0.1)        // 比例系数
    .withKI(0.01)       // 积分系数
    .withKD(0.001)      // 微分系数
    .withMaxI(0.5)      // 积分上限
    .withIZone(10.0)    // Izone阈值
    .withKS(0.05)       // 静态摩擦系数
    .withKV(0.12)       // 速度系数
    .withKA(0.01)       // 加速度系数
    .withOutputLimits(-1.0, 1.0); // 输出限幅
```

### 5. 输出分量读取

`PIDSVAController` 在每次 `calculate` 后会记录各分量贡献，可通过 getter 实时读取，用于遥测显示与整定诊断：

```java
// 读取最近一次总输出（限幅后）
double output = controller.getLastOutput();

// 读取各分量贡献（限幅前的原始值）
double pTerm = controller.getLastPTerm();  // kP * error
double iTerm = controller.getLastITerm();  // kI * integral
double dTerm = controller.getLastDTerm();  // kD * derivative
double sTerm = controller.getLastSTerm();  // kS * sign(参考速度)
double vTerm = controller.getLastVTerm();  // kV * 参考速度
double aTerm = controller.getLastATerm();  // kA * 参考加速度
```

整定时将各分量加入 FTC Dashboard plot，可直观判断哪一项主导输出、积分是否饱和、前馈是否充足。

---

## 典型用法示例

### 电机位置控制

```java
PIDSVAController motorController = new PIDSVAController().withSlot0(
    new SlotConfig()
        .withKP(0.05).withKI(0.0).withKD(0.001)
        .withKS(0.1).withKV(0.15).withKA(0.02)
        .withOutputLimits(-1.0, 1.0)
);

// 在循环中
double targetPos = 1000; // 目标编码器位置
double currentPos = motor.getCurrentPosition();
double output = motorController.calculate(targetPos, currentPos, dt, false);
motor.setPower(output);
```

### 电机速度控制

```java
PIDSVAController velocityController = new PIDSVAController().withSlot0(
    new SlotConfig()
        .withKP(0.01).withKI(0.001).withKD(0.0)
        .withKS(0.05).withKV(0.12).withKA(0.01)
        .withOutputLimits(-1.0, 1.0)
);

// 在循环中
double targetVel = 500; // 目标速度（编码器tick/s）
double currentVel = motor.getVelocity();
double output = velocityController.calculate(targetVel, currentVel, dt, true);
motor.setPower(output);
```

---

## 注意事项

1. `SlotConfig` 的默认输出限幅为 `-1.0 ~ 1.0`（对应 `setPower` 的功率范围）；如需电压控制（`setVoltage`），可用 `withOutputLimits(-14.0, 14.0)` 调整
2. `resetSlot` 会用新构建的 `SlotConfig` 整体替换原配置；未被 `withXxx` 覆盖的字段会回到默认值，更新个别参数时请先构建包含完整参数的 `SlotConfig`
3. 切换 slot 会重置积分、微分状态与 setpoint 历史
4. `PIDController` 没有 4 参数构造函数，需同时指定 `maxI` 和 `iZone` 时使用 5 参数版本
5. `PIDSVAController.calculate` 只接收 `setpoint` 与 `measurement`：SVA 前馈的参考速度/加速度由控制器内部按 `setpoint` 的变化率推导（速度闭环时 `setpoint` 即参考速度），因此**不要再传入实测速度**，否则会与微分项相互抵消
6. 调用 `calculate` 前必须已配置当前 slot（至少 `withSlot0(...)`），否则抛出 `IllegalArgumentException("Slot not configured")`