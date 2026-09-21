# MT1Localizer 设计与接口说明

本文说明 [MT1Localizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.java) 的职责与计划实现的功能，包括：

1. 从 Limelight 3A 获取 MegaTag1 的 botpose（原始视觉位姿）与质量指标；
2. 依据 [OneMap\_MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md) 的推导，用**单张水平地图**同时解算出 **HIVE 倾角** 与**机器人真实位姿**；
3. 向上层（EKF / 状态机）暴露统一接口。

> 说明：第 1~3 部分均已在 `.java` 中实现。HIVE 解算为**逐帧观测**，不含时间滤波、
> 不保存历史状态（滞回/保持由上层 RobotPosition 负责）。

***

## 一、在整体定位架构中的位置

```
Limelight3A ──► MT1Localizer ──► AdaptiveEKFLocalizer ──► RobotPosition ──► 状态机 / 底盘控制
  (视觉)          botpose 获取       EKF 融合                单例对外读取
                  hive 解算          (自适应 Q / R + 门控)
```

- MT1Localizer 是 `Localizer` 接口的实现（见 [Localizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/Localizer.java)），
  但它**只提供绝对位姿观测，不提供速度**，`update()` 恒返回零速度。
- 位姿与速度的融合由 [AdaptiveEKFLocalizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/FusionLocalizer/AdaptiveEKFLocalizer.java) 完成：
  里程计提供预测，MT1 提供观测，`getStdDevs()` 用于自适应构造观测噪声 R。
- HIVE 状态观测结果需经 EKF 透传给 [RobotPosition.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/RobotPosition/RobotPosition.java)
  持续估计，和自动状态机（见 [Plan.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Plan.md) 的 LAUNCH / WAIT\_CELL / EAT\_CELL 判断）。

***

## 二、botpose 获取（已实现）

### 2.1 数据链路

每帧调用 `update()`（`LIMELIGHT → LLResult`）：

```java
latestResult = limelight.getLatestResult();   // Limelight3A 内部缓存的最新一帧
if (latestResult == null || !latestResult.isValid()) { valid = false; return; }
botpose = latestResult.getBotpose();          // MegaTag1 解算出的机器人位姿 (Pose3D)
```

`getBotpose()` 返回的是 Limelight 依据**当前加载的 fmap** 解算出的场地位姿，字段定义：

| 字段                 | 类型                   | 说明           |
| :----------------- | :------------------- | :----------- |
| `getPosition()`    | `Position`           | 平移，单位**米**   |
| `getOrientation()` | `YawPitchRollAngles` | 姿态，角度单位**度** |

### 2.2 单位与坐标系转换

FTC 侧统一使用 **英寸 / 弧度**，转换常数 `M_TO_INCH = 39.37007874`：

$$
x_{\text{in}} = x_{\text{m}} \cdot 39.3701,\quad
y_{\text{in}} = y_{\text{m}} \cdot 39.3701,\quad
\theta_{\text{rad}} = \text{toRadians}(\text{yaw}_{\text{deg}})
$$

坐标系为 FTC 标准场地坐标系：原点在场地中心，x 向右、y 向前、z 向上，
对应的 Road Runner 二维位姿为 `Pose2d(x, y, heading)`。

### 2.3 质量指标（用于 EKF 自适应 R 与门控）

| 缓存字段              | `LLResult` 接口                           | 用途                                             |
| :---------------- | :-------------------------------------- | :--------------------------------------------- |
| `stdDevs[6]`      | `getStddevMt1()`                        | `{x, y, z, roll, pitch, yaw}`（米/度），构造 3×3 R 矩阵 |
| `tagCount`        | `getBotposeTagCount()`                  | 标签数，少标签时放大 R；`isReliable` 要求 ≥ 2               |
| `avgDist`         | `getBotposeAvgDist()`                   | 平均距离（米），远距离时 R 二次放大                            |
| `avgArea`         | `getBotposeAvgArea()`                   | 平均面积，辅助判断成像质量                                  |
| `span`            | `getBotposeSpan()`                      | 标签跨度（米），跨度越大姿态越准                               |
| `maxFiducialSkew` | 遍历 `getFiducialResults()` 的 `getSkew()` | 单标签倾斜度，越大越模糊（易发生翻转歧义）                          |
| `timestamp`       | `getTimestamp()`                        | 位姿时间戳（秒，Limelight 基准），供 EKF 对齐                 |
| `captureLatency`  | `getCaptureLatency()`                   | 捕获延迟（**毫秒**，SDK 文档口径），用于调试视觉滞后                  |

由 stdDev 派生的综合不确定度：

$$
\text{ambiguity} = \sqrt{\sigma_x^2 + \sigma_y^2}\ (\text{米}),\qquad
\text{angularAmbiguity} = \text{toRadians}(\sigma_{\text{yaw}})
$$

***

## 三、HIVE 角度解算与真实位姿复原（已实现）

算法推导见 [OneMap\_MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md)，
此处只给出工程实现所需的公式与判定流程。

### 3.1 问题描述

- Limelight 中加载的是 HIVE **水平参考状态**（$\theta = 0$）时的地图；
- 实际 HIVE 绕枢轴（过 $(0,0,h)$、方向 $\mathbf{u} = (0,1,0)$）旋转了 $\theta$；
- 因此 Limelight 返回的 botpose 是"被旋转过的"错误位姿 $T' = T_{\text{rot}}(\phi)\,T$，其中 $\phi = -\theta$；
- 需要在**单张地图**下，由 $T'$ 反解 $\phi$ 并复原真实位姿 $T$。

记 botpose 的平移为 $p' = (x', y', z')$（英制），姿态为 $R'$。

### 3.2 解算 $\phi$：利用机器人贴地约束

机器人在地面运动，真实高度 $z \approx 0$，代入 $z$ 分量方程得标准三角方程：

$$
A\sin\phi + B\cos\phi = C,\qquad A = x',\quad B = z' - h,\quad C = -h
$$

解的表达式为

$$
\phi = \arcsin\!\left(\frac{C}{\sqrt{A^2+B^2}}\right) - \operatorname{atan2}(B, A)
$$

工程实现要点：

1. **可解性检查**：若 $\left|C\right| > \sqrt{A^2+B^2}$，该帧无解，丢弃**该帧的 HIVE 观测**。
   注意 `isValid()` 与 `getPose()` **不受影响**：此时 `isHiveEstimated() = false`，
   `getPose()` 回退为未修正的原始位姿（受污染位姿，消费方须自行门控）。
2. **取根策略**：三角方程有两族解
   $\phi_1 = \alpha - \operatorname{atan2}(B,A)$ 与 $\phi_2 = \pi - \alpha - \operatorname{atan2}(B,A)$
   （$\alpha = \arcsin(C/R)$），二者恒相差 $\pi - 2\alpha$，一般情况下只有一族落入窗口。
   按 $|\phi| \le$ `hiveCellUpAngleDeg` 筛选；若两族都在窗口内则取 $|\phi|$ **较小**者。
   本类无状态，不用上一帧 $\phi$ 做连续性选择。
3. **姿态交叉校验（退化帧判别）**：$\phi$ 有两条相互独立的信息通路 ——
   位置通路（上式，唯一载体是 $z'$）与姿态通路：
   $$
   R' = R_y(\phi)\,R,\qquad \text{机器人贴地} \Rightarrow pitch' \approx \arcsin\!\big(\sin\phi \cdot \cos(yaw')\big)
   $$
   两者之差超过 `hivePitchCheckTolDeg` 即判为 $z'$ 退化帧，**整帧丢弃**。
   > 为什么必须做：由解算模型 $F(\phi) = x'\sin\phi + (z'-h)\cos\phi + h$ 恒有 $F(0) = z'$，
   > 因此 $|z'| \approx 0$ 时位置通路的解必然退化到 $\phi = 0$（$\theta = 0$），与 HIVE 真实倾角无关；
   > 而该帧的 `ambiguity` / 标签数等指标可能全部正常，无法用常规质量门剔除。
   > 姿态通路不依赖 $z'$，可作为独立仲裁。
   > 局限：前提是机器人贴地平放（在斜坡上会误拒）；航向接近 $\pm 90^\circ$ 时 $\cos(yaw') \to 0$，
   > 姿态通路本身不携带该旋转的信息，判据失去判别力。
4. **不做时间滤波**：$\phi$ 的 EMA / 中值滤波由上层负责（本类只输出逐帧观测）。

> 注：早期版本的「贴地残差校验」（由 $\phi$ 反算 $z$ 后与容差比较）在数学上恒等于 0 ——
> $\phi$ 是方程 $A\sin\phi + B\cos\phi = C$ 的解析根，反算 $z \equiv C + h = 0$ —— 不具任何判别力，
> 已由第 3 条的姿态交叉校验取代。

### 3.3 位置与姿态修正

位置（由 $p'$ 复原 $p$）：

$$
\begin{aligned}
x &= x'\cos\phi - z'\sin\phi + h\sin\phi \\
y &= y' \\
z &= x'\sin\phi + z'\cos\phi + h(1-\cos\phi)\ \ (\approx 0,\ \text{用于校验})
\end{aligned}
$$

姿态（由 $R'$ 复原 $R$）：

$$
R = R_y(-\phi)\,R',\qquad
R_y(-\phi) = \begin{bmatrix} \cos\phi & 0 & -\sin\phi \\ 0 & 1 & 0 \\ \sin\phi & 0 & \cos\phi \end{bmatrix}
$$

对外只输出二维位姿，yaw 由修正后的旋转矩阵提取。$R'$ 的第一列即机器人 $x$ 轴在场坐标系下的方向
（SDK 约定：内旋 yaw → pitch → roll，故 $R_{00} = \cos\psi'\cos P'$，$R_{10} = \sin\psi'\cos P'$，$R_{20} = -\sin P'$）：

$$
\text{yaw} = \operatorname{atan2}\big(R_{10},\ \cos\phi\,R_{00} - \sin\phi\,R_{20}\big)
$$

> 早期文档写作 $\text{yaw} \approx \operatorname{atan2}(R_{21}, R_{11})$，该式在机器人水平时恒为 0，属笔误，已按上式实现。

### 3.4 HIVE 状态判定

由 $\theta = -\phi$（HIVE 实际倾角）**逐帧瞬时**判定，**不含滞回**（本类不保存历史状态，
滞回/状态保持由上层 RobotPosition 负责）：

| 条件                                                       | 状态              | 说明                                                        |
| :------------------------------------------------------- | :-------------- | :-------------------------------------------------------- |
| 本帧无有效观测（无标签 / 无解 / 姿态交叉校验失败）                              | `UNKNOWN`       | 上层保持上一次已知状态，不切换                                           |
| $\lvert\theta\rvert \le \theta_{\text{down}}$            | `MIDDLE`        | HIVE 水平/中间（CELL 放下），两条通路的 $\phi$ 一致                        |
| $\theta_{\text{down}} < \lvert\theta\rvert < \theta_{\text{up}}$ 且 $\theta > 0$ | `AUDIENCE_UP`   | HIVE 稳定位于一侧；哪一侧抬升由 `hivePositiveAngleIsAudienceUp` 决定符号约定 |
| $\theta_{\text{down}} < \lvert\theta\rvert < \theta_{\text{up}}$ 且 $\theta < 0$ | `AUDIENCE_DOWN` | 另一侧抬升                                                     |

其中 $\theta_{\text{down}}$ = `hiveCellDownAngleDeg`，$\theta_{\text{up}}$ = `hiveCellUpAngleDeg`。
$\lvert\theta\rvert \ge \theta_{\text{up}}$ 的帧已在取根（§3.2-2）阶段被丢弃，不会进入本分类。

> 两个阈值同时承担两个职责：`hiveCellUpAngleDeg` 是取根窗口的**上界**（也是直接拒绝门限），
> `hiveCellDownAngleDeg` 是 MIDDLE 与"稳定位于一侧"的**分界**。实机标定时前者取略大于最大实际仰角，
> 后者取"确实水平"时残存抖动的上界。

### 3.5 使用约束

1. **只喂本方标签**：必须通过 `SetFiducialIDFiltersOverride` / pipeline 过滤，只保留本方 HIVE 的标签。
   红蓝 HIVE 混用会导致旋转轴不一致，解算失效（见 OneMap 文档结尾）。
2. **地图只保留水平参考状态一份**，无需按倾角动态上传 fmap（这一点优于 [map.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/map.md) 中的双地图切换方案）。
3. **$h$** **必须标定**：即枢轴中心相对场地地面的高度（英寸），误差会直接线性传入 $x$ 的修正量。
4. **可见性范围**：仅在 HIVE 前方标签可见时有效，符合 [Plan.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Plan.md) 中"定位无需覆盖全场，仅 hive 前方"的约定。
5. **Limelight 的 `snap to ground` 必须设为 `No`**：该选项开启时 Limelight 会把解算位姿强行压到地面平面上
   （等价于 $z' \equiv 0$），位置通路的倾角信息被完全抹掉，解算将**恒输出** $\theta = 0$。
   这是实机调试中确认过的失效原因，属配置问题而非算法问题。
6. **机器人需贴地平放**：姿态交叉校验（§3.2-3）以真实 pitch ≈ 0 为前提；机器人处于斜坡上时会被误判为退化帧。

***

## 四、对外接口

### 4.1 已实现（当前 `.java`）

`Localizer` 接口：

| 方法                | 返回               | 说明                               |
| :---------------- | :--------------- | :------------------------------- |
| `update()`        | `PoseVelocity2d` | 拉取最新帧；恒返回零速度（视觉不提供速度）            |
| `getPose()`       | `Pose2d`         | 全局位姿（英寸, 英寸, 弧度）；无效时返回 `(0,0,0)` |
| `setPose(Pose2d)` | `void`           | 空实现（视觉定位器不可写）                    |

扩展读取接口：

| 方法                                                              | 返回               | 说明                                                                |
| :-------------------------------------------------------------- | :--------------- | :---------------------------------------------------------------- |
| `getPoseArray()`                                                | `double[3]`      | `{x, y, theta}`（英寸, 英寸, 弧度）                                       |
| `getBotpose()`                                                  | `Pose3D`         | 原始 6DOF 位姿（米/度）                                                   |
| `getStdDevs()`                                                  | `double[6]`      | `{x,y,z,roll,pitch,yaw}` 副本（米/度）                                  |
| `getAmbiguity()`                                                | `double`         | 平面位置不确定度 $\sqrt{\sigma_x^2+\sigma_y^2}$（米）；无效时 `Double.MAX_VALUE` |
| `getAngularAmbiguity()`                                         | `double`         | yaw 不确定度（弧度）                                                      |
| `getTagCount()` / `getAvgDist()` / `getAvgArea()` / `getSpan()` | `int` / `double` | 质量指标                                                              |
| `getMaxFiducialSkew()`                                          | `double`         | 单标签最大倾斜度                                                          |
| `getTagIds()`                                                   | `String`         | 本帧识别到的所有 fiducial ID（逗号分隔），调试用：确认只含本方标签                          |
| `getTimestamp()` / `getCaptureLatency()`                        | `double`         | 时间戳（秒，Limelight 基准）/ 捕获延迟（**毫秒**）                                  |
| `isValid()`                                                     | `boolean`        | 当前是否拉到了有效结果（**不保证** HIVE 解算成功）                                    |
| `isReliable(double threshold)`                                  | `boolean`        | `valid && tagCount >= 2 && ambiguity < threshold`，用于 EKF 更新门控     |

HIVE 解算观测接口：

| 方法                            | 返回          | 说明                                                                      |
| :---------------------------- | :---------- | :---------------------------------------------------------------------- |
| `getHiveAngle()`              | `double`    | HIVE 倾角 $\theta = -\phi$（弧度），未解出时 `Double.NaN`                           |
| `getHiveState()`              | `HiveState` | `MIDDLE` / `AUDIENCE_UP` / `AUDIENCE_DOWN` / `UNKNOWN`（§3.4），无滞回           |
| `isHiveEstimated()`           | `boolean`   | 本帧是否成功解出 $\theta$；用于区分"确为 MIDDLE"与"没看见 / 被拒"                            |
| `getHivePitchCheckErrDeg()`   | `double`    | 姿态交叉校验偏差（度）；超过 `hivePitchCheckTolDeg` 的帧已被丢弃，无观测时 `NaN`（§3.2-3）        |
| `getRawPose()`                | `Pose2d`    | Limelight 原始（未修正）位姿，调试对比用                                               |
| `getRawZIn()` / `getRawPitch()` | `double`  | 原始位姿的 $z'$（英寸）/ pitch（弧度），退化帧诊断用（§3.2-3）                               |

### 4.2 尚未实现

| 方法                                                    | 返回     | 说明                                                                   |
| :---------------------------------------------------- | :----- | :------------------------------------------------------------------- |
| `setAlliance(boolean isRed)` 或 `setTagIds(int[] ids)` | `void` | 在本类内部只保留本方标签；当前依赖 Limelight 的 `SetFiducialIDFiltersOverride` / fmap 过滤 |
| `setMode(...)`                                        | `void` | 供 EKF 向 RobotPosition 透传 HIVE 状态（对应 `AdaptiveEKFLocalizer` 的 todo）     |

> 参数（$h$、两个角度阈值、校验容差、符号约定）不使用 setter 注入，统一由
> [HypParams.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Parameter/HypParams.java) 的静态字段提供，便于在 FTC Dashboard 上实时调参。

接口约定：

- `getPose()` 返回**修正后的真实位姿**，仍满足 `Localizer` 契约（英寸/弧度），EKF 无需改动即可受益；
  但 HIVE 解算失败时会**回退为未修正的受污染位姿**，因此消费方必须用
  `isValid() && isHiveEstimated()` 门控（见 §五）；
- `getHiveAngle()` / `getHiveState()` 的有效性由 `isHiveEstimated()` 守卫，
  上层状态机据此区分 `MIDDLE`（倾角确实很小）与 `UNKNOWN`（没看见 / 被拒）。

***

## 五、与 EKF 的协作流程（自适应 R 和门控）

[AdaptiveEKFLocalizer.update()](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/FusionLocalizer/AdaptiveEKFLocalizer.java#L182-L218) 中的调用顺序：

1. 里程计 `odom.update()` → 速度；
2. `adaptQ(dt)` → IMU 冲击/坡度自适应过程噪声；
3. `ekf.predict(...)`；
4. `mt1.update()` →
   - `mt1.isValid() && mt1.isHiveEstimated()` 才继续；否则该帧 `getPose()` 是未修正的受污染位姿，整帧丢弃；
   - `adaptR()` 用 `getStdDevs()` + `getAvgDist()` + `getTagCount()` 构造 3×3 R；
   - `ekf.gateVision(...)` 用马氏距离剔除离群观测；
   - `ekf.update(mt1.getPose(), mt1.getTimestamp())`。
5. 透传 `mt1.getHiveAngle()` / `mt1.getHiveState()`（待实现，见 `AdaptiveEKFLocalizer` 的 todo）。

对照定位器（`EKFLocalizer` / `UKFLocalizer`）不做马氏距离门控，但**同样要求**
`isValid() && isHiveEstimated()`，否则对照结果会包含受污染位姿而失去可比性。

因此 MT1Localizer 的改动**不能破坏**上述三组输出：`getPose()` / `getStdDevs()` / `getTimestamp()`。

***

## 六、参数与待办清单

需标定/配置的参数（均位于 [HypParams.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Parameter/HypParams.java)，可在 Dashboard 实时调整）：

| 参数                                | 含义                            | 备注                                  |
| :-------------------------------- | :---------------------------- | :---------------------------------- |
| `hivePivotHeightIn`               | 枢轴相对地面高度 $h$（英寸）              | 必须标定，误差会线性传入 $x$ 修正量                |
| `hiveCellDownAngleDeg`            | MIDDLE 的倾角上界（度）               | 同时也是"稳定位于一侧"的下界                     |
| `hiveCellUpAngleDeg`              | 稳定位于一侧的倾角上界（度），同时是**取根窗口上界**   | 超过即直接拒绝该帧，必须 > `hiveCellDownAngleDeg` |
| `hivePitchCheckTolDeg`            | 姿态交叉校验容差（度）                   | 位置通路与姿态通路 $\phi$ 之差超过该值 → 判为退化帧      |
| `hivePositiveAngleIsAudienceUp`   | 倾角符号约定（布尔）                    | 决定 $\theta > 0$ 对应哪一侧抬升，需实机标定         |

待办：

- [x] 实现 §3.2 的 $\phi$ 求解与取根；
- [x] 实现 §3.2-3 退化帧判别（姿态交叉校验）、实现 §3.3 位置/姿态修正并把 `getPose()` 切换到修正位姿；
- [x] 实现 §3.4 HIVE 状态判定（无滞回）与 §4.1 的 HIVE 观测接口；
- [x] 在 `MT1Test.java` / `FusionTestOpMode.java` 增加 $\theta$ / HIVE 状态 / 修正前后位姿 / Dashboard 轨迹对比；
- [ ] 在 `AdaptiveEKFLocalizer` 中透传 HIVE 状态到 [RobotPosition.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/RobotPosition/RobotPosition.java)，并在上层实现滞回 / 状态保持；
- [ ] 实机标定 §六 各参数（$h$ 与两个角度阈值）；
- [ ] `setAlliance` / `setTagIds` 标签过滤（当前依赖 Limelight 侧配置，见 §3.5-1）。

