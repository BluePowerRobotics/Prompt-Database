# ReplayBuffer 功能说明与接入指南

本文档说明 [ReplayBuffer.java](./ReplayBuffer.java) 的功能原理，并给出把「视觉时间戳回滚重放」接入**旧版定位链路**（`Processors/FusionLocalizer` + `utility/filter/EKF` + `utility/filter/UKF`）的完整改动步骤。

> 适用场景：另一个项目里已经有一份可用的 `FusionLocalizer`（EKF/UKF 融合定位），但**没有** `ReplayBuffer`。按本文改动后，即可获得与视觉延迟对齐的回滚重放能力。
>
> 若只需先跑通、暂不需要重放，可只做 [§5.1](#51-步骤-1复制-replaybufferjava) 与构造函数的 `allowReplay` 默认 `false` 部分；重放逻辑在 `allowReplay=false` 时完全不参与运算，行为与旧版逐位一致。

---

## 1. 背景与目标

### 1.1 要解决的问题：视觉链路延迟

Limelight 从曝光到 FTC 端拿到 `LLResult` 存在**端到端延迟**（曝光 + 传输 + 固件解算），典型几十毫秒量级。若不处理：

- `predict()` 用「当前时刻」推进状态；
- `update()` 却把「几十毫秒前拍的」视觉观测按当前时刻融合。

两者时间基准不一致 → 视觉修正被系统性地“往后错位”。在**匀速/低速**时可忽略，但在**加减速、急转**时表现为明显的滞后偏差（位姿被视觉“拖尾”）。

### 1.2 解决思路

参考 WPILib `PoseEstimator` 的“把带时间戳的观测对齐到其真实发生时刻再融合”，但本实现面向**递推滤波器（EKF/UKF）**，采用：

> **滤波器状态回滚 → 在观测时刻更新 → 再重放其后的里程计预测回到当前时刻**

与 `PoseEstimator`（对位姿缓冲做插值后施加统一变换）实现方式不同，二者不可互推。

### 1.3 参与文件清单

| 文件 | 角色 | 是否必需 |
|---|---|---|
| [`utility/filter/ReplayBuffer.java`](./ReplayBuffer.java) | 回滚重放缓冲区 | **新增** |
| `utility/filter/EKF/EKF.java` | 增加快照记录 + 回滚重放 | **修改** |
| `utility/filter/UKF/UKF.java` | 同上 | **修改** |
| `Processors/FusionLocalizer/EKFLocalizer.java` | 透传 `allowReplay` + 选对时间戳 | **修改** |
| `Processors/FusionLocalizer/UKFLocalizer.java` | 同上 | **修改** |
| `Processors/FusionLocalizer/AdaptiveEKFLocalizer.java` | 同上 | **修改** |
| `Processors/FusionLocalizer/AdaptiveUKFLocalizer.java` | 同上 | **修改** |
| `Processors/VisionLocalizer/MT1Localizer.java` | 提供 `System.nanoTime()` 基准的捕获时刻 | **修改（依赖）** |

> 依赖方向：`EKF`/`UKF`（包 `...utility.filter.EKF` / `...utility.filter.UKF`）`import ...utility.filter.ReplayBuffer`。`FusionLocalizer` 的四个类**不直接**依赖 `ReplayBuffer`，只调用 `ekf.isReplayEnabled()` 并选择时间戳。

---

## 2. ReplayBuffer 功能详解

### 2.1 它保存什么

每次 `predict()` 之后记录一个「状态快照」，包含：

| 字段 | 含义 | 用途 |
|---|---|---|
| `t` | 快照时间戳（秒，`System.nanoTime()` 基准） | `floor()`/`after()` 的时间索引 |
| `x` | 状态 `[x, y, θ]` 的副本 | 回滚恢复 |
| `P` | 状态协方差 3×3 的副本 | 回滚恢复 |
| `Q` | **传播到本快照所用的**过程噪声 3×3 的副本 | 重放时按历史值复用（见 §2.5） |
| `vx, vy, omega` | 由上一快照传播到本快照所用的**控制输入** | 重放段的速度输入 |

> 关键语义：`Snapshot(t_k, x_k, P_k, Q_k, u_k)` 中 `u_k` 表示“`t_{k-1}→t_k` 这一段所用的输入”，而 `Q_k` 表示“`t_{k-1}→t_k` 这一段所用的过程噪声”。因此 `floor()` 找到基准后，需要用它之后第一个快照的 `u`/`Q` 才能推进到观测时刻。

### 2.2 内部结构

```java
private final double duration;                          // 保留时长（默认 1.5 s）
private final ArrayDeque<Snapshot> samples = new ArrayDeque<>();
```

### 2.3 关键不变量与语义

| 方法 | 语义 | 边界行为 |
|---|---|---|
| `add(t, x, P, Q, vx, vy, omega)` | 追加快照 | **时间戳必须严格递增**：`t <= 队尾.t` 时直接丢弃（保护 `floor/after` 的单调假设） |
| 淘汰 | 保留最近 `duration` 秒 | `samples.size() > 2 && 队首.t < t - duration` 时弹出队首；**至少保留 2 个**以支持回滚 |
| `floor(t)` | 返回 `t_s <= t` 的**最新**快照 | 若所有快照都晚于 `t` → 返回 `null`（观测早于缓冲窗口） |
| `after(t)` | 返回 `t_s > t` 的全部快照，升序 | 无则返回空列表 |
| `clear()` | 清空 | `reset()` 时调用 |
| `size()` / `isEmpty()` | 调试/统计 | — |

### 2.4 拷贝语义

- 构造 `Snapshot` 时对 `x` 做 `clone()`、对 `P`/`Q` 做 `copy()`，与滤波器内部矩阵**解耦**，避免后续 `P = A·P·Aᵀ+...` 之类的重新赋值/原地修改污染历史。
- `getX()/getP()/getQ()` 同样返回副本。
- `refresh(x, P)` 在重放完成后**回写**该快照的状态与协方差（`Q` 不刷新，因为历史段的 Q 不可变），使缓冲与滤波器当前进度一致。

### 2.5 为什么回放必须用「历史 Q」而不是当前 Q

自适应策略下 `Q` 逐帧跳变（IMU 冲击/坡度检测 → `qBoost` 变化）。若用**当前** Q 回放整段历史：

- 会把“最新一帧的噪声量”重新灌进“过去某区间”的协方差；
- 重放结果与当初真实传播不一致 → 回滚-重放不闭合，累积系统偏差。

因此快照必须连同当时的 `Q` 一起保存，回放各段各用各段的历史 Q。注意：`R` 属于“本次观测自身”的噪声，与时间轴无关，回放时仍沿用当前帧由 `adaptR()` 算出的 `R`。

---

## 3. 回滚重放原理与时序

### 3.1 一帧的时序（启用重放）

```
EKF/UKF 侧                                      Localizer 侧
────────────────────────────────────────────   ─────────────────────────────
predict(u_now, t_now)                           odom.update() → 速度
  ├─ 推进状态/协方差
  └─ replayBuffer.add(t_now, x, P, Q, u_now)    mt1.update()
                                                └─ timestampNano = now - latency
update(z, t_obs)  ← t_obs = timestampNano       取得该帧 HIVE 修正位姿
  ├─ base = floor(t_obs)                         （仅 isValid && isHiveEstimated）
  ├─ 回滚到 base
  ├─ 用 tail[0].u / tail[0].Q 推进到 t_obs
  ├─ applyVisionUpdate(z)   ← 在 t_obs 处融合
  └─ 对 tail 中每个快照：
        applyMotion(s.u, s.t - prevT, s.Q)
        s.refresh(...)
```

### 3.2 `replayUpdate` 逐步说明

1. `base = replayBuffer.floor(t_obs)`：找到不晚于观测时刻的最近快照。
   - `base == null` → 观测早于缓冲窗口，**放弃**该次重放（返回 `false`，不更新 `lastUpdateTime`）。
2. 间隔保护：`t_obs - base.t > MAX_REPLAY_GAP (1.0 s)` → 放弃（预测停更/时钟异常），避免超大 `dt` 重放。
3. 回滚：`state = base.x`，`P = base.P`。
4. 推进到观测时刻：用 `after(base.t)` 的**第一个**快照的输入与历史 Q，`dt = t_obs - base.t`。
5. 在观测时刻执行一次标准观测更新 `applyVisionUpdate(z)`。
6. 重放回当前时刻：对 `after(base.t)` 的每个快照依次 `applyMotion(u, dt, Q)`，并 `refresh` 回写。

> 说明：因 `floor` 返回“≤ t_obs 的最近快照”，故 `after(base.t)` 中所有快照时间戳都 **严格大于** `t_obs`，第 4 步的 `dt>0`、第 6 步各段 `dt>0` 恒成立。

### 3.3 `allowReplay=false` 时的行为

完全等价于旧版：不创建缓冲、不记录快照，`update()` 走 `lastUpdateTime = timestamp; applyVisionUpdate(...)`，不进入 `replayUpdate`。

---

## 4. 接入总览（改动清单）

| 文件 | 改动摘要 |
|---|---|
| `ReplayBuffer.java` | 新增（直接拷贝，包名 `org.firstinspires.ftc.teamcode.utility.filter`） |
| `EKF.java` | 新增 `allowReplay`/`replayBuffer`/`MAX_REPLAY_GAP`；构造函数重载；`predict` 拆出 `applyMotion`；`update` 拆出 `applyVisionUpdate` 并加 replay 分支；新增 `replayUpdate`/`toColumn`/`isReplayEnabled`/`getReplayBufferSize`；`reset` 清空缓冲 |
| `UKF.java` | 同 `EKF.java`（`applyMotion` 用无迹变换实现，签名相同） |
| `EKFLocalizer.java` | 构造函数加 `allowReplay`；`update` 时间戳改为 `ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()` |
| `UKFLocalizer.java` | 同上 |
| `AdaptiveEKFLocalizer.java` | 同上（含 7 参完整构造） |
| `AdaptiveUKFLocalizer.java` | 同上 |
| `MT1Localizer.java` | 新增 `getTimestampNanoBase()`（把捕获时刻换算到 `System.nanoTime()` 基准）及 `isNewFrame` 去重 |

---

## 5. 详细步骤

### 5.1 步骤 1：复制 `ReplayBuffer.java`

把 [ReplayBuffer.java](./ReplayBuffer.java) 原样复制到目标项目：

```
<项目>\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\utility\filter\ReplayBuffer.java
```

要求：
- **包名不变**：`package org.firstinspires.ftc.teamcode.utility.filter;`
- 依赖 `org.ejml.simple.SimpleMatrix`（RoadRunner 已自带 EJML，无需额外依赖）。

### 5.2 步骤 2：修改 `EKF.java`

> 位置：`utility/filter/EKF/EKF.java`。若目标项目类名/包名不同，按同样逻辑迁移。

#### 5.2.1 新增 import 与字段

```java
import java.util.List;

import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer;

// ... 已有字段 ...

/** 上一次 update 的时间戳，用于拒绝过时观测（旧版若已有请保留） */
private Double lastUpdateTime = null;

/** 是否启用视觉时间戳回滚重放 */
private final boolean allowReplay;

/** 回滚重放缓冲区（仅 allowReplay 为 true 时创建，否则为 null） */
private final ReplayBuffer replayBuffer;

/** 回滚基准与观测时刻允许的最大间隔（秒），超过则放弃回滚 */
private static final double MAX_REPLAY_GAP = 1.0;
```

#### 5.2.2 构造函数重载

```java
public EKF(double initialX, double initialY, double initialTheta) {
    this(initialX, initialY, initialTheta, false);
}

public EKF(double initialX, double initialY, double initialTheta, boolean allowReplay) {
    // ... 原有的 state / P / Q / R 初始化保持不变 ...

    this.allowReplay = allowReplay;
    this.replayBuffer = allowReplay ? new ReplayBuffer() : null;
}
```

#### 5.2.3 `predict`：加快照记录，并把传播逻辑拆到 `applyMotion`

```java
public void predict(double vx, double vy, double omega, double timestamp) {
    // 首次调用：仅记录时间戳，不做预测
    if (lastPredictTime == null) {
        lastPredictTime = timestamp;
        if (allowReplay) {
            replayBuffer.add(timestamp, getPose(), P, Q, 0, 0, 0);
        }
        return;
    }

    double dt = timestamp - lastPredictTime;
    lastPredictTime = timestamp;

    // 异常 dt 保护
    if (dt <= 0 || dt > 1.0) {
        return;
    }

    applyMotion(vx, vy, omega, dt, Q);

    // 记录快照（连同本段所用的历史 Q 与输入）
    if (allowReplay) {
        replayBuffer.add(timestamp, getPose(), P, Q, vx, vy, omega);
    }
}
```

把原来 `predict` 中的「状态传播 + 雅可比 + 协方差传播」抽成私有方法，**多一个 `Qused` 参数**：

```java
private void applyMotion(double vx, double vy, double omega, double dt, SimpleMatrix Qused) {
    double x = state.get(0, 0);
    double y = state.get(1, 0);
    double theta = state.get(2, 0);

    double cosTheta = Math.cos(theta);
    double sinTheta = Math.sin(theta);

    double xNew = x + dt * (vx * cosTheta - vy * sinTheta);
    double yNew = y + dt * (vx * sinTheta + vy * cosTheta);
    double thetaNew = theta + dt * omega;

    SimpleMatrix A = new SimpleMatrix(new double[][]{
            {1, 0, dt * (-vx * sinTheta - vy * cosTheta)},
            {0, 1, dt * ( vx * cosTheta - vy * sinTheta)},
            {0, 0, 1                                     }
    });

    // 常规 predict 传当前 Q；回放时传该段快照记录的历史 Q
    SimpleMatrix Qdt = Qused.scale(dt);
    P = A.mult(P).mult(A.transpose()).plus(Qdt);

    state = new SimpleMatrix(new double[][]{
            {xNew},
            {yNew},
            {normalizeAngle(thetaNew)}
    });
}
```

#### 5.2.4 `update`：加 replay 分支，并把更新体拆到 `applyVisionUpdate`

```java
public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
    // 拒绝过时观测（不论是否启用重放都保留）
    if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
        return;
    }

    if (allowReplay) {
        if (replayUpdate(xMeas, yMeas, thetaMeas, timestamp)) {
            lastUpdateTime = timestamp;
        }
        return;
    }

    lastUpdateTime = timestamp;
    applyVisionUpdate(xMeas, yMeas, thetaMeas);
}
```

`applyVisionUpdate` 就是旧版 `update` 里「新息 → S → K → 状态 → 协方差」的主体，原样搬入：

```java
private void applyVisionUpdate(double xMeas, double yMeas, double thetaMeas) {
    SimpleMatrix z = new SimpleMatrix(new double[][]{{xMeas}, {yMeas}, {thetaMeas}});

    SimpleMatrix yInnov = z.minus(state);
    yInnov.set(2, 0, normalizeAngle(yInnov.get(2, 0)));

    SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
    SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

    state = state.plus(K.mult(yInnov));
    state.set(2, 0, normalizeAngle(state.get(2, 0)));

    SimpleMatrix I = SimpleMatrix.identity(3);
    P = I.minus(K.mult(H)).mult(P);
}
```

新增 `replayUpdate` 与 `toColumn`：

```java
private boolean replayUpdate(double xMeas, double yMeas, double thetaMeas, double timestamp) {
    ReplayBuffer.Snapshot base = replayBuffer.floor(timestamp);
    if (base == null) {
        return false;               // 观测早于缓冲窗口，无法回滚
    }
    if (timestamp - base.t > MAX_REPLAY_GAP) {
        return false;               // 间隔过大，放弃回滚
    }

    List<ReplayBuffer.Snapshot> tail = replayBuffer.after(base.t);   // t 严格 > 观测时刻

    // 1. 回滚到观测时刻之前的最近快照
    state = toColumn(base.getX());
    P = base.getP();

    // 2. 从该快照推进到观测时刻（用其后第一段的输入与历史 Q）
    if (!tail.isEmpty()) {
        ReplayBuffer.Snapshot first = tail.get(0);
        applyMotion(first.vx, first.vy, first.omega, timestamp - base.t, first.getQ());
    }

    // 3. 在观测时刻执行更新（R 沿用当前观测自身的 R）
    applyVisionUpdate(xMeas, yMeas, thetaMeas);

    // 4. 重放其后所有预测回到当前时刻，并刷新快照
    double prevT = timestamp;
    for (ReplayBuffer.Snapshot s : tail) {
        applyMotion(s.vx, s.vy, s.omega, s.t - prevT, s.getQ());
        prevT = s.t;
        s.refresh(getPose(), P);
    }
    return true;
}

private SimpleMatrix toColumn(double[] v) {
    return new SimpleMatrix(new double[][]{{v[0]}, {v[1]}, {v[2]}});
}
```

#### 5.2.5 `reset`：清空缓冲 + 重置时间戳

```java
public void reset(double x, double y, double theta) {
    // ... 原有的 state / P 重置保持不变 ...
    lastPredictTime = null;
    lastUpdateTime = null;
    if (replayBuffer != null) {
        replayBuffer.clear();
    }
}
```

#### 5.2.6 新增访问器

```java
/** @return 是否启用视觉时间戳回滚重放 */
public boolean isReplayEnabled() {
    return allowReplay;
}

/** @return 回滚缓冲中的快照数量（未启用重放时为 0） */
public int getReplayBufferSize() {
    return replayBuffer == null ? 0 : replayBuffer.size();
}
```

### 5.3 步骤 3：修改 `UKF.java`

与 `EKF.java` 改动**逐条对应**，差异仅在 `applyMotion` 的算法实现：

- `applyMotion(vx, vy, omega, dt, Qused)`：用 sigma 点无迹变换传播（sigma 点生成 → 逐点传播 → 圆形均值 → 加权协方差 + `Qused.scale(dt)`）。
- `update`/`replayUpdate`/`applyVisionUpdate`/`toColumn`/`reset`/构造/访问器：结构与 EKF 完全一致，只需把 `H`、`safeInvert`、`normalizeAngle` 换成 UKF 已有的实现。

> 参考：[UKF.java](../UKF/UKF.java) 中 `predict` / `applyMotion` / `update` / `applyVisionUpdate` / `replayUpdate`。

### 5.4 步骤 4：修改 `FusionLocalizer` 的四个定位器

对 `EKFLocalizer`、`UKFLocalizer`、`AdaptiveEKFLocalizer`、`AdaptiveUKFLocalizer` 做**两类**改动：

#### 5.4.1 构造函数透传 `allowReplay`

- 非自适应（`EKFLocalizer` / `UKFLocalizer`）：给 D2/D3 构造各加一个 `boolean allowReplay` 重载，最终传入滤波器：

```java
public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose,
                    boolean allowReplay) {
    this.ekf = new EKF(initialPose.position.x, initialPose.position.y,
                       initialPose.heading.toDouble(), allowReplay);   // ← 唯一变化点
    ekf.setQ(QbasePos, QbasePos, QbaseAngle);
    ekf.setR(RbasePos, RbasePos, RbaseAngle);
    this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
    this.mt1 = new MT1Localizer(limelight);
    this.useD3 = false;
}
```

- 自适应（`AdaptiveEKFLocalizer` / `AdaptiveUKFLocalizer`）：在原有的 `useD3`、`teamColor` 之后追加 `allowReplay`：

```java
public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                            String imuDeviceName, Pose2d initialPose, boolean useD3,
                            TeamColor teamColor, boolean allowReplay) {
    this.ekf = new EKF(initialPose.position.x, initialPose.position.y,
                       initialPose.heading.toDouble(), allowReplay);   // ← 唯一变化点
    // ... odom / mt1 / hubImu 初始化保持不变 ...
}
```

> 保留旧的构造重载，内部转调 `allowReplay=false`，保证既有调用点零改动。

#### 5.4.2 `update()` 里选择正确的时间戳基准

这是最关键的一处。**重放要求观测时间戳与 `predict` 快照同基准（`System.nanoTime()`）**。

- 未启用重放：沿用 Limelight 硬件时间戳（仅用于 `lastUpdateTime` 去重，不参与回滚）。
- 启用重放：改用 `mt1.getTimestampNanoBase()`。

四个定位器的 `update()` 中，把视觉更新那一段统一改为：

```java
if (mt1.isValid() && mt1.isHiveEstimated()) {
    ekf.setR(adaptR());                        // UKFLocalizer 为 ukf.setR(...)
    Pose2d visionPose = mt1.getPose();
    if (ekf.gateVision(                       // UKFLocalizer 无门控则省略此 if
            visionPose.position.x,
            visionPose.position.y,
            visionPose.heading.toDouble(),
            GATE_THRESHOLD)) {
        ekf.update(
                visionPose.position.x,
                visionPose.position.y,
                visionPose.heading.toDouble(),
                // 回滚重放：与 predict 快照同基准；否则沿用 Limelight 硬件时间戳
                ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
        );
    }
}
```

> 注意：
> - `EKFLocalizer` / `UKFLocalizer`（对照组）**不做**马氏门控，直接调用 `update`，但**同样**要改时间戳选择。
> - `AdaptiveUKFLocalizer` / `UKFLocalizer` 把 `ekf` 换成 `ukf`。

### 5.5 步骤 5：修改 `MT1Localizer`（时间戳基准依赖）

若旧版 `MT1Localizer` 只有 `getTimestamp()`（Limelight 硬件时钟），必须新增 `System.nanoTime()` 基准的捕获时刻，否则重放无法与预测快照对齐。

#### 5.5.1 新增字段

```java
/** 该帧捕获时刻的时间戳（秒），已换算到 System.nanoTime() 基准 */
private double timestampNano;

/** 上一物理帧的 Limelight 硬件时间戳（秒），用于判定是否为新帧；初值 NaN 保证首帧为新帧 */
private double lastFrameTimestamp = Double.NaN;
```

#### 5.5.2 `update()` 中计算

```java
double frameTimestamp = latestResult.getTimestamp();
captureLatency = latestResult.getCaptureLatency();

// 仅当硬件时间戳变化时才视为新物理帧：帧率低于循环频率时 getLatestResult() 会重复返回同一帧
boolean isNewFrame = frameTimestamp != lastFrameTimestamp;
lastFrameTimestamp = frameTimestamp;
timestamp = frameTimestamp;

// 换算到 System.nanoTime() 基准：本地收到结果的时刻减去捕获延迟
// 仅新帧刷新，重复读取同一帧时保持恒定，避免同一观测被重复融合 / 重复回滚
if (isNewFrame) {
    timestampNano = System.nanoTime() / 1e9 - captureLatency / 1000.0;
}
```

#### 5.5.3 新增访问器

```java
/** @return 该帧捕获时刻（秒），已换算到 System.nanoTime() 基准，供回滚重放使用 */
public double getTimestampNanoBase() {
    return timestampNano;
}
```

> 两个时钟不可直接相减：Limelight 硬件时间戳与 `System.nanoTime()` 分属不同基准。**只有 `timestampNano` 能与 EKF/UKF 内 `predict` 的快照时间戳比较**。

---

## 6. 验证与调试

1. **关闭重放对照**：同一 OpMode 同时构造 `allowReplay=false` 与 `true` 两份定位器，视觉有效时应位姿一致；机动（加减速/急转）时 `true` 的滞后应更小。
2. **缓冲规模**：打印 `localizer.getEKF().getReplayBufferSize()`（UKF 为 `getUKF()`）。稳定循环下应接近「`1.5 s ÷ 帧周期`」，例如 50 Hz ≈ 75 个。若长期为 0/1，说明 `predict` 未推进或时间戳未递增。
3. **回滚是否命中**：观测若早于缓冲窗口或间隔 > `MAX_REPLAY_GAP`，`replayUpdate` 返回 `false` 且**不更新** `lastUpdateTime`。可临时打日志确认不是长期丢弃。
4. **时间戳递增性**：`getTimestampNanoBase()` 必须随新物理帧递增；若恒定或倒退，去重（`timestamp <= lastUpdateTime`）会静默丢弃视觉，需要排查 `isNewFrame` 判据与固件时间戳。

---

## 7. 常见坑与注意事项

1. **必须同基准**：`predict` 用 `System.nanoTime()`，观测就必须用 `getTimestampNanoBase()`；否则 `floor()` 会拿硬件时间戳去比 `System.nanoTime()`，回滚永远失效或触发大 `dt`。
2. **不要共用硬件时间戳做回滚**：硬件时间戳仅用于 `lastUpdateTime` 去重。跨时钟比较是经典错误。
3. **同一物理帧只融合一次**：`timestampNano` 只在 `isNewFrame` 时刷新，配合滤波器的 `lastUpdateTime` 去重，避免 Limelight 帧率低于循环频率时同一观测被重复计入。
4. **`Q` 记录时机**：`predict` 必须在推进**之后**记录快照（`applyMotion` 之后的 `getPose()/P`），且记录的是「本段所用」的 `Q`/`u`，否则 `after()` 回放会错位一段。
5. **`R` 不随快照**：回放沿用当前帧的 `R`，不要把它也存进快照。
6. **线程模型**：`ReplayBuffer` 非线程安全，仅限 OpMode 单循环线程访问；不要在其它线程调用 `predict/update`。
7. **内存**：每快照持有 3×3 矩阵的若干副本，1.5 s @ 100 Hz ≈ 150 个快照，量级可忽略。
8. **回滚失败不算错**：观测若来自缓冲窗口之前，属于不可对齐的历史观测，丢弃是正确行为；此时该帧视觉被跳过，`predict` 照常进行。

---

## 8. 如何关闭重放（回退）

- 所有构造函数不传 `allowReplay`（或传 `false`）即可；此时：
  - `EKF/UKF` 不创建缓冲、`predict` 不记录、`update` 走非 replay 分支；
  - 观测时间戳回退为 `mt1.getTimestamp()`；
  - 运行时开销与行为与旧版一致。

也可以完全删除对 `ReplayBuffer` 的引用恢复旧版，但因改动均为**增量且默认关闭**，无需回退代码。

---

## 附：一句话总览

> `ReplayBuffer` 把每帧 `predict` 后的「状态/协方差/所用 Q/输入」快照存下来；收到带延迟的视觉观测时，先回滚到观测时刻融合、再重放其后的里程计预测回到当前时刻，从而消除视觉链路延迟造成的滞后。接入 = 复制 1 个新文件 + 给 EKF/UKF 加「记录快照 + 回滚重放」+ 给 4 个 Localizer 透传 `allowReplay` 并在 `update()` 中按 `isReplayEnabled()` 选择 `timestampNano` 基准 + 给 `MT1Localizer` 补 `getTimestampNanoBase()`。
