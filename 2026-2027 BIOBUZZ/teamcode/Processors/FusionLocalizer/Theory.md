# FusionLocalizer 原理说明

## 概述

`FusionLocalizer` 是一个多传感器融合定位器，用于 FTC 机器人竞赛。提供 **两种滤波器实现**：

| 实现 | 类名 | 滤波器 | 特点 |
|------|------|--------|------|
| **EKF** | `EKF` / `EKFLocalizer` / `AdaptiveEKFLocalizer` | 扩展卡尔曼滤波器 | 雅可比线性化，计算量小，成熟稳定 |
| **UKF** | `UKF` / `UKFLocalizer` / `AdaptiveUKFLocalizer` | 无迹卡尔曼滤波器 | Sigma 点无迹变换，无需雅可比，精度更高 |

它将 **里程计**（高频、短时精确、长时漂移）与 **Limelight MegaTag1 视觉定位**（低帧率、绝对位置、偶发不可用）的优势互补，输出平滑、鲁棒的全局位姿估计。

核心思想：**用里程计做帧间预测，用 Limelight 做绝对修正，用 Hub IMU 检测异常运动，用 Ambiguity 评估视觉置信度。**

### D2 与 D3 模式

| 特性 | D2 模式 (默认) | D3 模式 |
|------|---------------|---------|
| 里程计 | `PinpointLocalizer` (标准 2D) | `PinpointD3Localizer` (3D 斜坡补偿) |
| 斜坡补偿 | 无 | Hub IMU pitch/roll → 速度投影到水平面 |
| 所需硬件 | Pinpoint + Limelight | Pinpoint + Limelight + Hub IMU |
| 适用场景 | 平坦场地 | 有斜坡/坡道的场地 |

> 所有定位器类 (`EKFLocalizer`, `AdaptiveEKFLocalizer`, `UKFLocalizer`, `AdaptiveUKFLocalizer`) 均支持 D2/D3 模式切换。EKF 和 UKF 的 API 完全兼容，可即插即用替换。
>
> 5 维加速度计驱动变体（`AdaptiveEKF5DLocalizer` / `AdaptiveUKF5DLocalizer` 及其 D2 简化版 `*_D2`）见 [Theory5D.md](./Theory5D.md)。

> **D3 斜坡补偿符号约定**：D3 模式的速度投影采用 pitch 前仰为正、roll 左侧抬起为正，交叉项符号为**负号**（`v_x^{horiz} = v_x^{body}·cosθ - v_y^{body}·sinθ·sinφ`）。完整推导见 [`D3Localizer/Theory.md`](../D3Localizer/Theory.md)。

---

## 1. 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                     每帧 update() 流程                      │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   里程计 (D2: PinpointLocalizer / D3: PinpointD3Localizer) │
│     └─→ 局部速度 (vx, vy, ω)  [in/s]                      │
│        │                                                  │
│        ▼                                                  │
│   Hub IMU (BHI260IMU)                                     │
│     ├─ D2: roll/pitch 角加速度 → 场坐标 → 交叉映射 → adaptQ_x, adaptQ_y│
│     │  D3: roll/pitch 角速度 → 场坐标 → 交叉映射 → adaptQ_x, adaptQ_y │
│     └─ yaw 角加速度 → adaptQ_θ                             │
│        │                                                  │
│        ▼                                                  │
│   adaptQ(dt) ──→ SimpleMatrix Q (3x3, in²/s)              │
│        │                                                  │
│        ▼                                                  │
│   EKF/UKF.setQ(Q) + EKF/UKF.predict(vx, vy, ω, timestamp)   │
│        │           状态 x, y 单位: 英寸                    │
│        ▼                                                  │
│   Limelight MT1 ──→ {x_m, y_m} [米] ──→ ×39.37 ──→ {x, y} [英寸] │
│        │                                                  │
│        ▼                                                  │
│   adaptR() ──→ SimpleMatrix R (3x3, in²)                  │
│        │                                                  │
│        ▼                                                  │
│   EKF/UKF.setR(R) + EKF/UKF.update(x, y, θ, timestamp)      │
│                    (仅当视觉有效)                           │
└──────────────────────────────────────────────────────────┘
```

### 状态定义

```
状态向量:  x = [x, y, θ]ᵀ
  - x, y: 全局坐标系下的机器人位置 (英寸)
  - θ:    全局朝向 (弧度, [-π, π])

控制输入:  u = [vx, vy, ω]ᵀ  (Pinpoint 提供的机器人局部速度)
  - vx: 前进速度 (in/s)
  - vy: 横移速度 (in/s)
  - ω:  角速度 (rad/s)

观测:     z = [x_m, y_m, θ_m]ᵀ  (Limelight 测得的全局位姿)
  - 原始单位: 米 -> 在传入滤波器前转换为英寸
```

---

## 2. EKF 预测步骤 (Predict)

### 2.1 非线性运动模型

将 Pinpoint 提供的局部速度转换到全局坐标系：

```
x' = x + Δt · (vx·cosθ - vy·sinθ)
y' = y + Δt · (vx·sinθ + vy·cosθ)
θ' = θ + Δt · ω
```

### 2.2 雅可比矩阵

计算状态传播函数对状态的偏导，用于协方差传播：

```
      ┌                                        ┐
      │ 1    0    Δt·(-vx·sinθ - vy·cosθ)      │
A =   │ 0    1    Δt·( vx·cosθ - vy·sinθ)      │
      │ 0    0    1                             │
      └                                        ┘
```

### 2.3 协方差传播

```
P' = A · P · Aᵀ + Q · Δt
```

其中 `Q · Δt` 体现了过程噪声随时间累积的特性：Δt 越大，预测的不确定度增长越多。

---

## 3. EKF 更新步骤 (Update)

### 3.1 新息 (Innovation)

```
y = z - H·x          (H = I, 观测直接对应状态)
```

角度新息需要归一化到 `[-π, π]`，避免 ±2π 跳变导致错误。

### 3.2 卡尔曼增益

```
S = H·P·Hᵀ + R
K = P·Hᵀ · S⁻¹
```

### 3.3 状态与协方差更新

```
x = x + K·y
P = (I - K·H)·P
```

---

## 2b. UKF 预测步骤 (Predict)

UKF 不使用雅可比矩阵，而是通过 **无迹变换 (Unscented Transform)** 直接传播非线性分布。

### 2b.1 UKF 参数

UKF 需要预先设定三个参数，用于控制 sigma 点的分布：

| 参数 | 值 | 含义 |
|------|-----|------|
| **α** (alpha) | 1.0 | Sigma 点散布因子，控制点到均值的距离。α 越小，点越集中 |
| **β** (beta) | 2.0 | 最优参数，对于高斯分布 β=2 是最优的 |
| **κ** (kappa) | 0.0 | 次级缩放参数，通常设为 0 或 3-n |
| **n** | 3 | 状态维度 = [x, y, θ] |
| **λ** | 0 | λ = α²(n+κ) - n = 1·3 - 3 = 0 |
| **n+λ** | 3 | 缩放因子，用于计算 sigma 点偏差和权重 |

### 2b.2 Sigma 点生成

从当前状态估计 `x̂` 和协方差 `P` 生成 **2n+1 = 7 个** sigma 点：

```
1. 计算缩放协方差矩阵: S = (n+λ) · P = 3 · P

2. Cholesky 分解: S = L · Lᵀ
   其中 L 是下三角矩阵 (3×3)，使用 EJML DecompositionFactory_DDRM.chol() 计算

3. 生成 sigma 点:
   X₀   = x̂                              (均值点)
   Xᵢ   = x̂ + L_colᵢ                     (i = 1, 2, 3)
   Xᵢ₊₃ = x̂ - L_colᵢ                     (i = 1, 2, 3)
```

**Sigma 点权重**:

| 索引 | 均值权重 wm | 协方差权重 wc | 说明 |
|------|------------|--------------|------|
| 0 | λ/(n+λ) = **0** | λ/(n+λ) + (1-α²+β) = **2** | 中心点 |
| 1..6 | 1/(2(n+λ)) = **1/6** | 1/(2(n+λ)) = **1/6** | 外围点 |

> **注意**: wc[0] = 2 是合理的。对于 n=3, α=1, β=2，权重和 Σwc = 2 + 6×(1/6) = 3，满足 Σwc = n + λ 的关系。

### 2b.3 非线性运动模型传播

将每个 sigma 点通过同一非线性运动模型：

```
X̂ᵢ[0] = Xᵢ[0] + Δt · (vx·cos(Xᵢ[2]) - vy·sin(Xᵢ[2]))
X̂ᵢ[1] = Xᵢ[1] + Δt · (vx·sin(Xᵢ[2]) + vy·cos(Xᵢ[2]))
X̂ᵢ[2] = Xᵢ[2] + Δt · ω
```

### 2b.4 加权均值 (圆形均值)

```
x̂̄[0] = Σ wm[i] · X̂ᵢ[0]       (位置: 普通加权平均)
x̂̄[1] = Σ wm[i] · X̂ᵢ[1]       (位置: 普通加权平均)
x̂̄[2] = atan2( Σ wm[i]·sin(X̂ᵢ[2]), Σ wm[i]·cos(X̂ᵢ[2]) )   (角度: 圆形均值)
```

**为什么使用圆形均值？** 角度是周期性的 ([-π, π])。如果 sigma 点分布在 ±π 附近，普通算术平均会产生错误结果。圆形均值 `atan2(Σsin, Σcos)` 正确处理了角度环绕。

### 2b.5 协方差传播

```
P̄ = Σ wc[i] · (X̂ᵢ - x̂̄)(X̂ᵢ - x̂̄)ᵀ + Q · Δt
```

每个 sigma 点的偏差中，角度差需要归一化到 `[-π, π]` 后再计算外积。

---

## 3b. UKF 更新步骤 (Update)

由于观测模型是线性的 (`H = I`，观测直接对应状态)，UKF 更新步骤可以简化，但仍使用完整的无迹变换以保证数值一致性。

### 3b.1 观测 Sigma 点

由于 `H = I`，观测 sigma 点 `Zᵢ` 等同于状态 sigma 点 `Xᵢ`：

```
Zᵢ = H · Xᵢ = Xᵢ
```

### 3b.2 新息 (Innovation)

```
预测观测均值 ẑ = Σ wm[i] · Zᵢ  (= x̂̄, 与状态均值相同)
新息 y_innov = z - ẑ

角度新息归一化到 [-π, π]
```

### 3b.3 新息协方差与交叉协方差

```
S = Σ wc[i] · (Zᵢ - ẑ)(Zᵢ - ẑ)ᵀ + R    (新息协方差)
Pxz = Σ wc[i] · (Xᵢ - x̂̄)(Zᵢ - ẑ)ᵀ      (交叉协方差)
```

### 3b.4 卡尔曼增益与状态更新

```
K = Pxz · S⁻¹
x̂ = x̂̄ + K · y_innov
```

### 3b.5 Joseph 形式协方差更新

UKF 使用 **Joseph 形式** 更新协方差，而非 EKF 的 `(I-KH)` 形式：

```
P = P̄ - K · S · Kᵀ
```

**为什么用 Joseph 形式？**

| 形式 | 公式 | 特点 |
|------|------|------|
| EKF 标准 | `P = (I - K·H)·P` | 简洁，但数值舍入误差可导致 P 不对称 |
| Joseph 形式 | `P = P - K·S·Kᵀ` | 天然对称，保证正半定性，更稳定 |

UKF 在 Joseph 更新后还额外执行对称性强制：

```
P = (P + Pᵀ) / 2
```

### 3b.6 Cholesky 分解与数值稳定性

Sigma 点生成依赖 Cholesky 分解 `L·Lᵀ = (n+λ)·P`。当协方差矩阵 `P` 因浮点舍入误差变为非正定时，Cholesky 会失败。UKF 使用**三级正则化降级策略**：

```
尝试 1: 直接分解 scale·P
   ↓ 失败
尝试 2: 分解 scale·P + 1e-6 · I (加微小正则化)
   ↓ 仍失败
尝试 3: 分解 scale·P + 1e-3 · I (加大正则化)
   ↓ 仍失败
Fallback: 返回对角矩阵 sqrt(trace(scale·P)/3) · I
```

这个策略确保任何情况下都能生成有效的 sigma 点，不会因数值问题导致滤波器崩溃。

---

## 3c. EKF 与 UKF 对比

### 3c.1 算法差异总览

| 维度 | EKF | UKF |
|------|-----|-----|
| **非线性处理** | 一阶泰勒展开 (雅可比 `A = ∂f/∂x`) | Sigma 点无迹变换，精确到二阶矩 (高斯三阶) |
| **雅可比矩阵** | 需手动推导 `A[0][2]=Δt·(-vx·sinθ-vy·cosθ)` | ❌ 不需要 |
| **状态传播** | 1 次 | 7 次 (2n+1 个 sigma 点) |
| **协方差传播** | `P = A·P·Aᵀ + Q·dt` | Sigma 点加权外积 + Q·dt |
| **卡尔曼增益** | `K = P·(P+R)⁻¹` | `K = Pxz·S⁻¹` |
| **协方差更新** | `P = (I-K·H)·P` | Joseph 形式 `P = P - K·S·Kᵀ` |
| **角度均值** | 无 (仅 1 个状态点) | 圆形均值 `atan2(Σsin, Σcos)` |
| **矩阵分解** | 无 | Cholesky 分解 (3×3) |
| **计算量** | ~15 次 3×3 矩阵乘法 | ~15 次 3×3 矩阵乘法 + 7 次状态传播 + Cholesky |
| **FTC 帧耗时** | < 50 μs | < 200 μs (仍然远低于 1 帧预算) |

### 3c.2 何时使用 UKF

| 场景 | 推荐 | 原因 |
|------|------|------|
| 高速旋转 (omega > 2π rad/s) | **UKF** | EKF 线性化误差在 cos/sin 快速变化时累积 |
| 碰撞后恢复 | **UKF** | UKF 更精确地传播被碰撞扰动后的协方差 |
| 平坦场地、低速 | EKF | 两者精度相当，EKF 更简洁 |
| 计算资源极度受限 | EKF | UKF 的 Cholesky 分解有微小开销 |
| 调试/教学 | EKF | 雅可比矩阵可手算验证，便于理解 |

### 3c.3 精度差异的理论解释

EKF 将非线性运动模型 `f(x)` 在估计点处一阶泰勒展开：

```
EKF:  f(x̂ + δx) ≈ f(x̂) + A·δx
```

这意味着 `f` 的曲率信息丢失了。当 `θ` 变化较大时（如 10ms 内旋转 0.5 rad），`cos(θ+δ)` 和 `cos(θ) - sin(θ)·δ` 之间的误差可达 ~0.12 rad，在位置传播中转化为 ~0.12·v·Δt 英寸的偏差。

UKF 通过 7 个 sigma 点直接采样非线性函数，捕获了 `cos/sin` 的曲率：

```
UKF:  P̄ = Σ wc[i] · (f(Xᵢ) - x̂̄)(f(Xᵢ) - x̂̄)ᵀ
```

这等价于对 `f(x)` 做统计线性回归，最小化的是整个分布范围内的线性化误差，而非单点误差。

### 3c.4 API 兼容性

EKF 和 UKF 的公开 API 完全一致：

```java
// 两种滤波器使用方式完全相同
EKF filter1 = new EKF(x, y, theta);
UKF filter2 = new UKF(x, y, theta);

filter1.predict(vx, vy, omega, timestamp);  // 与 filter2 完全相同
filter1.update(xMeas, yMeas, thetaMeas, timestamp);  // 与 filter2 完全相同
filter1.setQ(SimpleMatrix Q);  // 与 filter2 完全相同
filter1.setR(SimpleMatrix R);  // 与 filter2 完全相同

// 定位器封装也可以即插即用
AdaptiveEKFLocalizer ekfLoc = new AdaptiveEKFLocalizer(...);
AdaptiveUKFLocalizer ukfLoc = new AdaptiveUKFLocalizer(...);  // 仅类型名不同
```

自适应 Q/R 逻辑 (IMU 冲击检测、D2/D3 策略、Limelight stdDev 三段式映射、boost/decay 动态调节) 在两套实现中**完全相同**，因为自适应逻辑只调节 Q/R 矩阵的值，不依赖滤波器内部实现。

---

## 4. 自适应 Q 调节 (Hub IMU)

### 4.1 动机

当机器人发生碰撞、急停、被其他机器人撞击时，**里程计会瞬间产生不可靠的位移**（编码器打滑、IMU 震荡）。如果此时 EKF 仍然信任里程计预测，融合位姿会偏离真实位置。

此外，在 **D3 模式**下，机器人经过斜坡时坡度变化会导致pinpoint腾空，也需要适当提高 Q 值。

**解决方案**：使用 Hub IMU (BHI260IMU) 检测异常运动，**各方向独立**调节 Q 倍增因子。D2 和 D3 使用不同的检测策略。不确定性方向与旋转轴垂直：pitch 绕 body Y 轴 → X 方向轮子腾空，roll 绕 body X 轴 → Y 方向轮子腾空。

### 4.2 D2 模式：仅角加速度 (冲击检测)

Hub IMU 无法直接提供线性加速度，因此使用 roll/pitch 角加速度作为机器人受到冲击的间接指标，再通过当前航向角将体坐标系扰动转换到场坐标系。

**体坐标系定义**:
- roll = `xRotationRate` → 绕 body X 轴（左右）旋转
- pitch = `yRotationRate` → 绕 body Y 轴（前后）旋转
- yaw = `zRotationRate` → 绕 body Z 轴（垂直）旋转

**交叉映射原理**: 不确定性方向与旋转轴垂直。pitch 绕 body Y 轴 → X 方向轮子腾空，roll 绕 body X 轴 → Y 方向轮子腾空。

| 方向 | 传感器 | 物理量 | 物理含义 |
|---|---|---|---|
| **x** | Hub IMU pitch 角加速度 | `\|fieldY\|` (rad/s²) | 前向/后向碰撞（pitch 绕 Y → X 方向腾空，取 fieldY 分量） |
| **y** | Hub IMU roll 角加速度 | `\|fieldX\|` (rad/s²) | 侧向碰撞（roll 绕 X → Y 方向腾空，取 fieldX 分量） |
| **θ** | Hub IMU yaw 角加速度 | `\|Δω_z/Δt\|` (rad/s²) | 旋转方向的碰撞 |

数据来源：
- `getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate` → roll 角速度
- `getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate` → pitch 角速度
- `getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate` → yaw 角速度

体坐标系 → 场坐标系旋转公式 (Rz(θ)):
```
rollAccel  = (rollRate  - lastRollRate)  / dt
pitchAccel = (pitchRate - lastPitchRate) / dt

fieldX = rollAccel · cos(heading) - pitchAccel · sin(heading)
fieldY = rollAccel · sin(heading) + pitchAccel · cos(heading)

// 交叉映射: pitch 绕 Y → X 方向腾空 → X 不确定度取 fieldY
//           roll  绕 X → Y 方向腾空 → Y 不确定度取 fieldX
qBoostX = updateBoost(qBoostX, |fieldY|, ANGULAR_ACCEL_THRESHOLD)
qBoostY = updateBoost(qBoostY, |fieldX|, ANGULAR_ACCEL_THRESHOLD)
```

### 4.3 D3 模式：角速度 (坡度变化检测)

D3 模式使用 `PinpointD3Localizer` 进行 3D 斜坡补偿，里程计本身已对斜坡做了速度投影校正。但坡度**变化时**（如上坡过渡到平地），里程计的速度估计仍可能产生瞬时偏差。D3 使用**角速度**检测坡度变化，同样以交叉映射分配到各方向：

| 信号 | 检测内容 | 物理含义 | Boost 上限 |
|------|----------|----------|------------|
| **场地坐标系角速度** | `\|fieldVelY\|`, `\|fieldVelX\|` | 坡度变化（机器人正在倾斜/恢复），交叉映射 | 10x |
| **yaw 角加速度** | `\|Δω_yaw/Δt\|` | 旋转冲击 | 10x |

> **重要**: yaw 角速度与坡度无关，不参与 D3 的坡度变化检测。yaw 仅使用角加速度来检测旋转冲击。

D3 将角速度旋转到场地坐标系后交叉映射：

```
// 角速度旋转到场地坐标系 (Rz(θ))
fieldVelX = rollRate · cos(heading) - pitchRate · sin(heading)
fieldVelY = rollRate · sin(heading) + pitchRate · cos(heading)

// 交叉映射: pitch 绕 Y → X 方向腾空 → X 不确定度取 fieldVelY
//           roll  绕 X → Y 方向腾空 → Y 不确定度取 fieldVelX
qBoostX = updateBoost(qBoostX, |fieldVelY|, ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX)
qBoostY = updateBoost(qBoostY, |fieldVelX|, ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX)
```

**设计意图**：旋转到场地坐标系后交叉映射，各方向独立判断。例如机器人面向前方上坡时，pitch 角速度主要映射到场地 Y 方向，交叉映射后提升 X 方向的 Q；侧面碾压时 roll 角速度主要映射到场地 X 方向，交叉映射后提升 Y 方向的 Q。

### 4.4 Q 调整策略

每个方向独立执行相同的 boost/decay 逻辑：

```
if magnitude > threshold:
    qBoost = min(Q_BOOST_MAX, qBoost × (1 + magnitude / threshold))
else:
    qBoost = max(1.0, qBoost × Q_DECAY)

Q_diag[i] = Q_base × qBoost[i]
```

- **冲击时 (D2)**：角加速度导致 qBoost 指数增长，冲击幅度越大提升越快
- **坡度变化时 (D3)**：角速度导致 qBoost 指数增长 (最大 10x)
- **平稳后**：qBoost 指数衰减（每帧 ×0.85），回到基线状态

最终 `adaptQ(dt)` 返回一个 3x3 对角 `SimpleMatrix`，直接传入 `EKF/UKF.setQ(SimpleMatrix)`。

### 4.5 参数调优指南

| 参数 | 推荐值 | 含义 | 模式 |
|------|--------|------|------|
| `ANGULAR_ACCEL_THRESHOLD` | 5.0 rad/s² | roll/pitch 角加速度触发阈值 | D2 |
| `JERK_THRESHOLD` | 4.0 rad/s² | yaw 角加速度触发阈值 | D2, D3 |
| `ANGULAR_VEL_THRESHOLD` | 1.0 rad/s | roll/pitch 角速度触发阈值 (坡度变化) | D3 |
| `VEL_BOOST_MAX` | 10.0 | 角速度最大 Q 倍增因子 (坡度变化) | D3 |
| `Q_BOOST_MAX` | 10.0 | 最大 Q 倍增因子 | D2 |
| `Q_DECAY` | 0.85 | 每帧衰减系数 | D2, D3 |

---

## 5. 自适应 R 调节 (Limelight stdDev)

### 5.1 动机

Limelight MegaTag1 的定位精度在不同条件下差异很大：
- **多标签、近距离、光照好** → 精度高（stddev ~0.01m）
- **单标签、远距离、倾斜角大** → 精度低（stddev ~0.1m+）
- **遮挡、运动模糊** → 完全不可用

**解决方案**：利用 Limelight 固件输出的 `stddevMt1` 各方向分量，**独立调节 R_x, R_y, R_θ**。每个方向的不确定度只影响该方向的观测噪声。

### 5.2 stdDev 来源与单位转换

```
double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (米, 度)
```

来自 `LLResult.getStddevMt1()` — Limelight 固件内部 MegaTag1 算法直接输出的标准偏差。

**位置 vs 角度使用不同阈值**:
- **x, y**: stdDev 从米 → 英寸 (×39.37)，与位置阈值比较
- **θ (yaw)**: stdDev 从度 → 弧度，与角度专用阈值比较

### 5.3 R 调整策略 (每方向独立，去除上界)

单一方向 std → R 采用**无上界的线性映射**（超过高阈值后继续线性放大，不再截断在 `R_MAX_SCALE`，9/2 改进）：

```
if std <= LOW:                          R = 0.01
else:  t = (std - LOW) / (HIGH - LOW)
       R = 0.01 × (1 + t × (R_MAX_SCALE - 1))   # 无上界, 线性继续放大
```

- **位置方向 (x, y)**：`std_inch = std_meter × 39.37`，阈值用 `STD_LOW_INCH` / `STD_HIGH_INCH`。
- **角度方向 (θ)**：`std_rad = Math.toRadians(std_yaw_deg)`，阈值用 `STD_LOW_ANGLE` / `STD_HIGH_ANGLE`。

位置方向的 R 还会额外乘以两个缩放因子，以应对远离 AprilTag / 标签过少时视觉误差急剧放大的现象（9/2 实验结论）：

```
rX     = mapStdToR(stdX_inch)  × distFactor × tagFactor
rY     = mapStdToR(stdY_inch)  × distFactor × tagFactor
rTheta = mapStdToRAngle(stdYaw_rad)      # 角度不乘距离/标签因子
```

各缩放因子定义：

- **距离缩放 `distFactor`**：`dist = mt1.getAvgDist()`（米），当 `dist > DIST_REF_M` 时按二次放大 `(dist / DIST_REF_M)²`，否则为 1。远离 tag 时视觉位置误差近似随距离平方增长，故用二次项。
- **标签数缩放 `tagFactor`**：`tags = mt1.getTagCount()`，当 `tags >= TAG_REF` 时为 1；当 `0 < tags < TAG_REF` 时按 `TAG_REF / tags` 放大（上限 `TAG_SCALE_MAX`）；当 `tags <= 0` 时取 `TAG_SCALE_MAX`。标签越少位姿解算越不可靠，故放大 R。

最终 `adaptR()` 返回一个 3x3 对角 `SimpleMatrix`，直接传入 `EKF/UKF.setR(SimpleMatrix)`。

### 5.4 参数调优指南

| 参数 | 推荐值 | 含义 |
|---|---|---|
| `STD_LOW_INCH` | 2.0 in (≈0.05m) | 位置：低于此值完全信任视觉（R=0.01） |
| `STD_HIGH_INCH` | 6.0 in (≈0.15m) | 位置：线性放大区间上边界（超出后继续线性放大） |
| `STD_LOW_ANGLE` | 0.035 rad (≈2°) | 角度：低于此值完全信任视觉 |
| `STD_HIGH_ANGLE` | 0.175 rad (≈10°) | 角度：线性放大区间上边界 |
| `R_MAX_SCALE` | 20.0 | std→R 线性映射斜率（**不再截断上界**） |
| `DIST_REF_M` | 1.0 m | 距离缩放参考距离（超出后二次放大） |
| `TAG_REF` | 2.0 | 标签数缩放参考标签数 |
| `TAG_SCALE_MAX` | 4.0 | 标签过少时 R 的最大放大倍数 |
| `GATE_THRESHOLD` | 4.0 | 马氏距离门控阈值（无量纲，越大越宽松） |

### 5.5 马氏距离门控 (离群点拒绝)

即使 R 已按距离/标签数放大，远处视觉解仍可能产生巨大误差，直接更新会把滤波器带偏。因此每次视觉更新前先用**马氏距离门控**判断该观测是否离群：

```
S  = P + R                    (观测模型 H = I → 新息协方差)
d² = (z - x̂)ᵀ · S⁻¹ · (z - x̂)
if d² <= GATE_THRESHOLD² :  接受更新
else                      :  拒绝 (该帧视觉作废)
```

- 角度新息在计算前归一化到 `[-π, π]`。
- 被拒绝的帧只跳过视觉更新，predict 照常进行，滤波器继续依赖里程计直到下一次可信观测。
- 该门控逻辑封装在滤波器基类的 `gateVision()` 中，各定位器在 `update()` 视觉分支内先门控、通过后再 `update()`。

---

## 6. 自适应策略的协同效果

### 典型场景 1：正常行驶

```
里程计精度高, Limelight 偶尔更新
→ Hub IMU 角速度/角加速度平稳 → qBoost_x/y/θ ≈ 1.0
→ Q = Q_base (正常), R 根据 stdDev 缩放
→ 滤波器主要依赖里程计预测, 视觉仅做缓慢修正
```

### 典型场景 2：机器人碰撞 (D2)

```
Hub IMU 检测到 roll/pitch 角加速度 >> 5 rad/s² 或 yaw 角加速度 >> 4 rad/s²
→ 对应方向的 qBoost 瞬间提升到 3~10x
→ 该方向 Q 变大, 滤波器不再信任里程计预测
→ 下次视觉更新时, 该方向卡尔曼增益 K 很大
→ 位姿迅速收敛到视觉观测值
→ 各方向独立: 仅受冲击的方向提升 Q, 其他方向保持正常
```

### 典型场景 3：坡度变化 (D3 专属)

```
机器人上坡或下坡时, roll/pitch 角速度 >> 1 rad/s
→ D3 角速度检测触发 → qBoost_x/y 提升到 2~10x
→ Q 适度增大, 里程计在坡度过渡期的不可靠性被考虑
→ 坡度稳定后 (角速度下降), qBoost 快速衰减 → 回到正常 Q
```

### 典型场景 4：Limelight 部分遮挡（单标签，远距离）

```
MT1 仍然有效, 但各方向 stdDev 不均匀
→ 例: stdX=0.08m(3.1in), stdY=0.12m(4.7in), stdYaw=8°
→ R_x ≈ 0.05, R_y ≈ 0.12, R_θ ≈ 0.17
→ 视觉对 x 方向修正较强, 对 y 和 θ 方向修正较弱
→ 避免单标签在各方向均匀拉偏
```

### 典型场景 5：视觉完全丢失

```
MT1 长时间无效 (无标签或遮挡)
→ 只有 predict 步骤, 没有 update 步骤
→ 滤波器纯靠里程计, 协方差 P 随时间增长
→ 一旦视觉恢复 (stdDev 低), K 自动增大, 快速修正
```

---

## 7. 单位一致性

滤波器内部状态统一使用 **英寸 + 弧度**，所有传感器数据在输入前完成单位转换：

| 输入 | 原始单位 | 转换 | 输入滤波器时单位 |
|---|---|---|---|
| Pinpoint 速度 (vx, vy) | in/s | 无需转换 | in/s |
| Pinpoint 角速度 (ω) | rad/s | 无需转换 | rad/s |
| Limelight 位置 (x, y) | 米 (m) | × 39.37 | 英寸 (in) |
| Limelight 朝向 (θ) | 度 (°) | × π/180 | 弧度 (rad) |
| MT1 stdDev 位置 (x, y) | 米 (m) | × 39.37 | 英寸 (in) |
| MT1 stdDev 角度 (yaw) | 度 (°) | × π/180 | 弧度 (rad) |
| **滤波器输出位姿** | — | — | **英寸 + 弧度** |

> **角度阈值独立**：`mapStdToR` 使用 `STD_LOW_INCH`/`STD_HIGH_INCH` 比较位置 stdDev；`mapStdToRAngle` 使用 `STD_LOW_ANGLE`/`STD_HIGH_ANGLE` 比较角度 stdDev，避免弧度值与英寸值误比较。

转换常数定义：
```java
private static final double M_TO_INCH = 39.37007874;
```

---

## 8. API 数据流

```
adaptQ(double dt)
  │
  ├─ D2 模式:
  │   ├─ hubImu.getRobotAngularVelocity() → rollRate(xRotationRate), pitchRate(yRotationRate)
  │   │   → rollAccel, pitchAccel → 旋转到场坐标 (Rz(θ))
  │   │   → fieldX = rollAccel·cosH - pitchAccel·sinH
  │   │   → fieldY = rollAccel·sinH + pitchAccel·cosH
  │   │   → 交叉映射: qBoostX ← |fieldY|, qBoostY ← |fieldX|
  │   └─ hubImu.getRobotAngularVelocity() → yawRate → |α_yaw| → qBoostTheta
  │
  ├─ D3 模式:
  │   ├─ hubImu.getRobotAngularVelocity() → rollRate, pitchRate
  │   │   ├─ fieldVelX = rollRate·cosH - pitchRate·sinH
  │   │   ├─ fieldVelY = rollRate·sinH + pitchRate·cosH
  │   │   → 交叉映射: qBoostX ← |fieldVelY|, qBoostY ← |fieldVelX|
  │   └─ hubImu.getRobotAngularVelocity() → yawRate
  │       └─ |α_yaw| → updateBoost → qBoostTheta  (yaw角速度不参与)
  │
  └─→ 构造 SimpleMatrix(3x3) 对角 Q (in²/s)  →  return Q

adaptR()
  │
  ├─ mt1.getStdDevs()  →  stdX_m, stdY_m, stdYaw_deg
  ├─ stdX_m × 39.37 → stdX_inch → mapStdToR  → × distFactor × tagFactor → rX
  ├─ stdY_m × 39.37 → stdY_inch → mapStdToR  → × distFactor × tagFactor → rY
  └─ Math.toRadians(stdYaw_deg) → mapStdToRAngle → rTheta (角度, 不缩放)
  │
  └─→ 构造 SimpleMatrix(3x3) 对角 R (in²)  →  return R

视觉更新门控 (mt1.isValid() && mt1.isHiveEstimated()):
  ├─ isHiveEstimated() = false → 该帧 HIVE 倾角无解或被拒
  │    (无标签 / 无解 / 取根超 CellUpAngle / 姿态交叉校验不通过 / 枢轴高度未标定)
  │    → mt1.getPose() 回退为未经修正的受污染位姿, 整帧丢弃
  └─ 通过 → adaptR() → setR(R) → gateVision(x, y, θ, GATE_THRESHOLD) 通过才 update(x, y, θ, timestamp)

对照组 (EKFLocalizer / UKFLocalizer):
  无马氏距离门控, 但同样要求 isValid() && isHiveEstimated()

EKF/UKF:
  setQ(SimpleMatrix Q)   — 直接接收完整 Q 矩阵
  setR(SimpleMatrix R)   — 直接接收完整 R 矩阵
  setQ(double, double, double)  — 标量接口 (保留)
  setR(double, double, double)  — 标量接口 (保留)
```

---

## 9. 时间戳管理与数据一致性

### 9.1 两种时间基准

| 来源 | 时间基准 | 用途 |
|---|---|---|
| `System.nanoTime()` | Java 单调时钟 | predict 的 dt 计算 |
| `LLResult.getTimestamp()` | Limelight 硬件时间戳 | update 的观测时间 |

### 9.2 防过时机制

滤波器内部维护 `lastUpdateTime`，拒绝 timestamp 小于等于上次更新的观测，避免网络延迟或帧率错乱导致的数据回退。

### 9.3 首次 Predict 处理

首次调用 `predict` 时只记录时间戳不做状态传播，避免 dt 为 0 导致除以零。

---

## 10. 参考

- Kou & Haggenmiller (2023), "Extended Kalman Filter State Estimation for Autonomous Competition Robots"
- Wan & van der Merwe (2001), "The Unscented Kalman Filter" — UKF 经典论文
- Julier & Uhlmann (1997), "A New Extension of the Kalman Filter to Nonlinear Systems" — 无迹变换理论基础
- Limelight MegaTag1 文档: https://docs.limelightvision.io/docs/docs-megatag
- GoBilda Pinpoint 文档: https://docs.gobilda.com/pinpoint-odometer
- BHI260IMU 文档: https://www.bosch-sensortec.com/products/smart-sensors/bhi260/
- EJML (Efficient Java Matrix Library): https://ejml.org