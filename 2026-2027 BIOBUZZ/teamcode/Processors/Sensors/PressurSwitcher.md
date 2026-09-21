# FTC 压力传感器制作与使用指南

本文档介绍如何使用薄膜压敏电阻（FSR）和电位器制作一个可调灵敏度的压力传感器，并将其封装为一个易于使用的开关量模块（`PressureSwitcher`），适用于 FTC 竞赛中的压力检测场景。

---

## 1. 概述

- **目标**：将压力（按压）转换为数字开关信号（`true`/`false`），并支持通过硬件电位器或软件接口调节触发灵敏度。
- **硬件原理**：FSR 与电位器串联分压，模拟输入引脚测量电位器两端电压。压力增大时，FSR 阻值减小，输出电压升高。
- **软件封装**：`PressureSwitcher` 类读取模拟电压，经 EMA 滤波后与阈值比较，实时输出按压状态。

---

## 2. 硬件制作

### 2.1 所需元件

- 薄膜压敏电阻（FSR） ×1
- 电位器（建议 10kΩ 线性） ×1
- 杜邦线/导线若干
- FTC 主控（REV Control Hub / Expansion Hub） ×1

### 2.2 电路原理图

```
VCC (3.3V) ── FSR ──●── 电位器 ── GND
                     |
                 模拟输入引脚
                     (AO)
```

- **FSR** 连接在 VCC 与信号节点之间。
- **电位器** 连接在信号节点与 GND 之间（作为可变下拉电阻）。
- **模拟输入引脚** 连接在 FSR 与电位器的公共节点。

### 2.3 连接步骤

1. **FSR 连接**：将 FSR 的一个引脚接至主控的 **3.3V** 引脚，另一个引脚接至信号线（即模拟输入引脚）。
2. **电位器连接**：将电位器的一个固定端接 **GND**，另一个固定端悬空（或与滑动端短接，但通常只需使用滑动端和其中一个固定端）。将电位器的**滑动端（中间脚）** 接至信号线（与 FSR 相连）。
3. **模拟输入**：将信号线接到主控的任一模拟输入端口（如 `Analog Input 0`）。
4. **固定元件**：确保所有连接牢固，避免接触不良。

> **⚠️ 注意**：REV Hub 的模拟输入参考电压为 **3.3V**，请勿接入高于 3.3V 的电压，以免损坏主控。

### 2.4 电位器调节灵敏度原理

- **顺时针旋转**（通常增大阻值）→ 信号点电压 **升高**（相同压力下输出更高）→ 更容易触发阈值 → **灵敏度提高**。
- **逆时针旋转**（减小阻值）→ 信号点电压 **降低** → 更难触发 → **灵敏度降低**。

电位器提供了硬件层面的粗调，配合软件阈值微调，可快速适配不同压力场景。

---

## 3. 软件使用 —— `PressureSwitcher` 类

该类封装了模拟输入读取、滤波与阈值判断，所有电压均**归一化到 [0,1]**（以 3.3V 为满量程），方便统一处理。

### 3.1 类结构概览

| 方法 / 字段 | 说明 |
|------------|------|
| `PressureSwitcher(HardwareMap, String)` | 构造，使用默认阈值 (0.5) 和 EMA 系数 (0.8) |
| `PressureSwitcher(HardwareMap, String, double, double)` | 构造，指定 alpha 和阈值 |
| `update()` | 采样、滤波并更新按压状态，**每帧必须调用** |
| `isPressed()` | 返回最新按压状态（`true`=压力超过阈值） |
| `getVoltage()` | 返回归一化滤波电压（EMA 输出） |
| `getRawVoltage()` | 返回归一化原始电压（未滤波） |
| `setThreshold(double)` | 动态调整归一化阈值 [0,1] |
| `getThreshold()` | 获取当前阈值 |
| `setAlpha(double)` | 动态调整 EMA 平滑系数 (0,1] |
| `reset()` | 重置滤波器状态 |

### 3.2 在 OpMode 中使用

#### 初始化
```java
import org.firstinspires.ftc.teamcode.Processors.Sensors.PressureSwitcher;

public class MyOpMode extends LinearOpMode {
    private PressureSwitcher pressure;

    @Override
    public void runOpMode() {
        // 使用配置中的设备名称 "pressure_sensor"
        pressure = new PressureSwitcher(hardwareMap, "pressure_sensor");

        // 可选：设置自定义阈值（归一化，如 0.6 表示 60% 满量程）
        // pressure.setThreshold(0.6);

        waitForStart();

        while (opModeIsActive()) {
            // 必须每帧调用 update()
            pressure.update();

            // 读取按压状态
            if (pressure.isPressed()) {
                // 执行动作，如启动电机、改变指示灯等
            }

            // 调试信息
            telemetry.addData("Voltage (filtered)", pressure.getVoltage());
            telemetry.addData("Threshold", pressure.getThreshold());
            telemetry.addData("Pressed", pressure.isPressed());
            telemetry.update();
        }
    }
}
```

#### 动态调节阈值（例如用手柄控制）
```java
// 在手柄循环中
if (gamepad1.a) {
    pressure.setThreshold(0.7);     // 按 A 设为高阈值
}
if (gamepad1.b) {
    pressure.setThreshold(0.4);     // 按 B 设为低阈值
}
// 用左摇杆微调（上推增加阈值）
double delta = -gamepad1.left_stick_y * 0.005;
double newThreshold = pressure.getThreshold() + delta;
pressure.setThreshold(Math.max(0, Math.min(1, newThreshold)));
```

#### 动态调节 EMA 平滑系数
```java
// 用右摇杆调节滤波强度（推荐范围 0.3 ~ 0.95）
double alpha = 0.5 + 0.45 * (gamepad1.right_stick_y + 1) / 2;
pressure.setAlpha(alpha);
```

### 3.3 参数调优建议

- **阈值（threshold）**：建议在典型压力下，读取 `getVoltage()` 的值，然后将阈值设为比该值略低（例如低 0.05~0.1），确保可靠触发。
- **EMA 系数（alpha）**：值越接近 1 响应越快，但抗噪性差；值越小越平滑，但延迟增加。推荐起始值 **0.8**，根据实际抖动情况调整。

---

## 4. 校准与调试流程

### 4.1 硬件校准（电位器）

1. 将程序阈值设为固定值（如 0.5）。
2. 对 FSR 施加你希望触发开关的**标准压力**。
3. 观察 telemetry 中 `getVoltage()` 的数值。
4. 旋转电位器，使该压力下的电压**刚好超过**阈值（例如阈值 0.5，则调至电压约 0.55）。
5. 此时，任何大于标准压力的输入都会触发 `isPressed() == true`。

### 4.2 软件微调

- 若硬件调节不便，可直接在程序中修改 `setThreshold()` 的值，实现远程校准。
- 推荐结合游戏手柄的按键或摇杆实时微调阈值，极大方便赛前调试。

### 4.3 调试信息输出

建议在 telemetry 中至少显示以下数据：
- 滤波电压（`getVoltage()`）
- 当前阈值（`getThreshold()`）
- 按压状态（`isPressed()`）

这样可直观了解传感器工作状态。

---

## 5. 注意事项

- **电压范围**：确保模拟输入电压不超过 3.3V。若电路设计不当导致电压过高，可串联限流电阻保护。
- **信号噪声**：FSR 信号易受机械振动和电磁干扰，EMA 滤波能有效抑制，但若噪声过大，可适当降低 `alpha` 值（如 0.6）。
- **电源稳定性**：建议使用主控的 3.3V 供电，避免使用外部电源引起参考电压不一致。
- **电位器选型**：推荐使用 10kΩ 线性电位器，阻值过大或过小可能影响分压范围和线性度。
- **非线性**：FSR 电阻与压力呈非线性关系，但作为开关量使用，只需确保在目标压力点附近有足够电压变化即可，无需精确标定。

---

## 6. 完整代码文件

`PressureSwitcher.java` 已提供，需确保项目中包含 `utility.filter.EMA` 类（简易指数移动平均滤波器）。

EMA 类参考实现（若不存在）：
```java
package org.firstinspires.ftc.teamcode.utility.filter;

public class EMA {
    private double alpha;
    private double filteredValue;
    private boolean initialized;

    public EMA(double alpha) {
        setAlpha(alpha);
        reset();
    }

    public double update(double raw) {
        if (!initialized) {
            filteredValue = raw;
            initialized = true;
        } else {
            filteredValue = alpha * raw + (1 - alpha) * filteredValue;
        }
        return filteredValue;
    }

    public double getFilteredValue() { return filteredValue; }

    public void setAlpha(double alpha) {
        if (alpha <= 0 || alpha > 1) throw new IllegalArgumentException("alpha must be in (0,1]");
        this.alpha = alpha;
    }

    public double getAlpha() { return alpha; }

    public void reset() {
        filteredValue = 0;
        initialized = false;
    }
}
```

---

## 7. 总结

通过上述硬件电路与 `PressureSwitcher` 类，你可以在 FTC 机器人上快速实现一个可靠、可调的压力开关。硬件电位器提供物理粗调，软件接口提供远程微调，两者结合可灵活适应不同比赛任务需求。

如有问题，建议先检查硬件连接，再用 telemetry 观察电压值，逐步校准阈值。祝比赛顺利！