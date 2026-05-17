# 基于pinpoint与视觉定位校准的融合定位算法
在平面场地中，小车位姿由位置和朝向描述。记世界坐标系原点固定，小车在世界系下的位置为 $\mathbf{p} = \begin{bmatrix} x \\ y \end{bmatrix}$，航向角为 $\theta$（正前方与世界 $x$ 轴夹角，逆时针为正）。其朝向对应的旋转矩阵为  

$$
\mathbf{R}(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta &  \cos\theta
\end{bmatrix}.
$$

任意向量在小车坐标系与世界坐标系之间的变换满足 $\mathbf{p}_{\text{world}} = \mathbf{R}\,\mathbf{p}_{\text{robot}}$。小车的完整位姿可用状态向量  

$$
\mathbf{X} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
$$

表示。定位的核心任务是根据传感器测量实时估计 $\mathbf{X}$。

## 传统 Pinpoint 定位原理

Pinpoint（或通用轮式里程计+IMU）系统通过读取编码器脉冲和陀螺仪角速度，基于机器人运动学模型进行航迹推算。设第 $k$ 次采样间隔内，机器人相对自身坐标系获得的位移增量为 $\Delta \mathbf{u} = \begin{bmatrix} \Delta x_{o} \\ \Delta y_{o} \\ \Delta \theta_{o} \end{bmatrix}$，其中 $\Delta x_o,\Delta y_o$ 为平面位移（由编码器差分计算），$\Delta \theta_o$ 为航向增量（由陀螺仪积分）。该增量与当前朝向 $\theta_{k-1}$ 组合，得到世界系下的位姿递推：

$$
\mathbf{X}_k = \mathbf{X}_{k-1} + 
\begin{bmatrix}
\cos\theta_{k-1} \cdot \Delta x_o - \sin\theta_{k-1} \cdot \Delta y_o \\
\sin\theta_{k-1} \cdot \Delta x_o + \cos\theta_{k-1} \cdot \Delta y_o \\
\Delta \theta_o
\end{bmatrix}.
$$

由于编码器分辨率有限、轮子打滑及 IMU 漂移，增量 $\Delta\mathbf{u}$ 含有噪声，其协方差记为 $\mathbf{Q}$。纯里程计递推会导致误差无界累积，长时间定位精度差。

## 基于标记点方位角的视觉定位

为解决纯里程计漂移问题，引入环境中位置已知的标记点(Tag)，通过视觉测量标记点相对于小车的方位角，直接计算世界坐标。

### 可解性推导

设有 $n$ 个标记点，其世界坐标已知为 $\mathbf{P}_i = \begin{bmatrix} X_i \\ Y_i \end{bmatrix}$，$i=1,\dots,n$。小车可测量标记点相对自身正前方的水平方位角 $\alpha_i$（逆时针为正），则在小车坐标系中，指向标记点的单位方向向量为  

$$
\mathbf{v}_i = \begin{bmatrix} \cos\alpha_i \\ \sin\alpha_i \end{bmatrix}.
$$

将其变换到世界系，得到世界系下的方向向量 $\mathbf{R}\mathbf{v}_i$。设标记点 $i$ 到小车的距离为 $d_i>0$，则有  

$$
\mathbf{P}_i = \mathbf{p} + d_i\,\mathbf{R}\mathbf{v}_i, \qquad i=1,\dots,n. \tag{1}
$$

方程 (1) 包含未知量 $\mathbf{p},\theta$ 以及 $n$ 个距离 $d_i$，共 $n+3$ 个自由度。每个标记点提供两个标量方程，共 $2n$ 个约束。自由度归零的必要条件是  

$$
2n \ge n+3 \;\Longrightarrow\; n \ge 3.
$$

故至少需要 $3$ 个标记点才能唯一确定位姿。

### 消去距离——直线约束

记 $\beta_i = \theta + \alpha_i$。将 (1) 写成分量形式并消去 $d_i$：用 $(X_i - x)$ 和 $(Y_i - y)$ 分别乘以 $\sin\beta_i$ 和 $\cos\beta_i$ 并相减，得  

$$
(X_i - x)\sin\beta_i - (Y_i - y)\cos\beta_i = 0,
$$

整理为  

$$
x\sin\beta_i - y\cos\beta_i = X_i\sin\beta_i - Y_i\cos\beta_i \triangleq C_i. \tag{2}
$$

方程 (2) 的几何意义是：小车位置必在一条由标记点位置和方位角确定的直线上。

### 用两标记点解位置（含未知 $\theta$）

取 $i=1,2$，得到关于 $x,y$ 的线性方程组  

$$
\begin{cases}
x\sin\beta_1 - y\cos\beta_1 = C_1, \\
x\sin\beta_2 - y\cos\beta_2 = C_2.
\end{cases} \tag{3}
$$

系数行列式为  

$$
\Delta = \sin\beta_1(-\cos\beta_2) - \sin\beta_2(-\cos\beta_1) = \sin(\alpha_2 - \alpha_1).
$$

当 $\sin(\alpha_2 - \alpha_1) \neq 0$ 时，解得  

$$
x = \frac{-C_1\cos\beta_2 + C_2\cos\beta_1}{\sin(\alpha_2 - \alpha_1)},\qquad
y = \frac{C_2\sin\beta_1 - C_1\sin\beta_2}{\sin(\alpha_2 - \alpha_1)}. \tag{4}
$$

此时 $(x,y)$ 已表示为 $\theta$ 的函数。

### 确定航向角 $\theta$

将 (4) 代入第三个标记点对应的方程 (2)（$i=3$），得到相容条件  

$$
C_1\sin(\alpha_2 - \alpha_3) + C_2\sin(\alpha_3 - \alpha_1) + C_3\sin(\alpha_1 - \alpha_2) = 0. \tag{5}
$$

将 $C_i = X_i\sin(\theta+\alpha_i) - Y_i\cos(\theta+\alpha_i)$ 展开，利用  

$$
\sin(\theta+\alpha_i) = \sin\theta\cos\alpha_i + \cos\theta\sin\alpha_i,\quad
\cos(\theta+\alpha_i) = \cos\theta\cos\alpha_i - \sin\theta\sin\alpha_i,
$$

代入 (5) 并合并同类项，可得关于 $\theta$ 的齐次线性方程  

$$
A\sin\theta + B\cos\theta = 0, \tag{6}
$$

其中系数由标记点坐标和方位角完全确定：

$$
\begin{aligned}
A &= \sum_{(i,j,k)} \bigl( X_i\cos\alpha_i + Y_i\sin\alpha_i \bigr) \sin(\alpha_j - \alpha_k), \\
B &= \sum_{(i,j,k)} \bigl( X_i\sin\alpha_i - Y_i\cos\alpha_i \bigr) \sin(\alpha_j - \alpha_k),
\end{aligned}
$$

求和下标 $(i,j,k)$ 取循环轮换 $(1,2,3),\,(2,3,1),\,(3,1,2)$。

由 (6) 解得两个候选航向（相差 $\pi$）：

$$
\theta = \operatorname{atan2}(-B,\,A) \quad\text{或}\quad \theta = \operatorname{atan2}(B,\,-A). \tag{7}
$$

### 求解距离与位置

选定 $\theta$ 后，任取一对标记点（例如 1 和 2）计算距离  

$$
d_1 = \frac{(X_2 - X_1)\sin(\theta+\alpha_2) - (Y_2 - Y_1)\cos(\theta+\alpha_2)}{\sin(\alpha_2 - \alpha_1)}. \tag{8}
$$

若 $d_1 < 0$，表明该 $\theta$ 使标记点位于后方，不符合实际（标记点须在前方），应取另一分支。最终位置为  

$$
\begin{aligned}
x &= X_1 - d_1\cos(\theta + \alpha_1), \\
y &= Y_1 - d_1\sin(\theta + \alpha_1). \tag{9}
\end{aligned}
$$

### 退化情况

- **所选两标记点与小车共线**：若 $\alpha_2 - \alpha_1 = k\pi\;(k\in\mathbb{Z})$，则 $\sin(\alpha_2 - \alpha_1)=0$，系数矩阵奇异，无法用该对标记点表达位置。若三个标记点均与小车共线，任意一对皆失效。
- **三个标记点共线**：此时方程 (6) 中 $A = B = 0$，$\theta$ 任意解，定位无穷多。
- **小车位于三标记点的外接圆（危险圆）**：设三不共线标记点确定的外接圆为 $\Gamma$。若小车在 $\Gamma$ 上，由圆周角定理，方程 (6) 中 $A = B = 0$，航向不可定；或在对称分布下出现两个 $d_i>0$ 的有效解，无法唯一区分。
- **小车与某标记点重合**：此时 $d_i = 0$，方位角 $\alpha_i$ 无定义，方程失效。

若使用 $4$ 个标记点，且保证任意三点不共线、四点不共圆，则任取三个点得一个危险圆，四个组合给出四个不同的圆。平面上任意点至多位于其中一个圆上，故总存在至少一组三个点可避开退化。实际系统可利用冗余进行最小二乘优化，进一步提高鲁棒性。

## 视觉与 Pinpoint 的误差状态卡尔曼滤波融合

视觉定位提供绝对位姿观测，但更新频率低且可能暂时失效；Pinpoint 提供高频、平滑的里程计增量，但会漂移。采用误差状态卡尔曼滤波（ESKF）将二者融合，可发挥各自优势。

### 状态定义

设世界系下名义状态（即滤波器输出的最优估计）为  

$$
\mathbf{X} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}.
$$

真实状态 $\mathbf{X}_{\text{true}}$ 与名义状态间存在误差 $\delta\mathbf{X} = \begin{bmatrix} \delta x \\ \delta y \\ \delta \theta \end{bmatrix}$，满足 $\mathbf{X}_{\text{true}} = \mathbf{X} \oplus \delta\mathbf{X}$（此处 $\oplus$ 表示位姿复合）。滤波器估计误差的均值为 $\mathbf{0}$，协方差为 $\mathbf{P} \in \mathbb{R}^{3\times 3}$。

### 预测步骤（里程计驱动）

Pinpoint 提供里程计增量 $\Delta\mathbf{u}_k = \begin{bmatrix} \Delta x_{o} \\ \Delta y_{o} \\ \Delta \theta_{o} \end{bmatrix}_k$（自身坐标系）。利用当前航向 $\theta_{k-1}$ 变换到世界系：

$$
\Delta\mathbf{X}_k = 
\begin{bmatrix}
\cos\theta_{k-1} \cdot \Delta x_{o} - \sin\theta_{k-1} \cdot \Delta y_{o} \\
\sin\theta_{k-1} \cdot \Delta x_{o} + \cos\theta_{k-1} \cdot \Delta y_{o} \\
\Delta \theta_{o}
\end{bmatrix}.
$$

名义状态更新为  

$$
\mathbf{X}_{k} = \mathbf{X}_{k-1} + \Delta\mathbf{X}_k. \tag{10}
$$

在误差状态层面，线性化的误差动力学为  

$$
\delta\mathbf{X}_k \approx \mathbf{F}\,\delta\mathbf{X}_{k-1} + \mathbf{B}\,\mathbf{w}_k,
$$

其中 $\mathbf{F} = \mathbf{I}_{3\times 3}$（近似），$\mathbf{B}$ 为控制输入雅可比，$\mathbf{w}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})$ 为里程计过程噪声。协方差预测为  

$$
\mathbf{P}_k = \mathbf{F} \mathbf{P}_{k-1} \mathbf{F}^\top + \mathbf{B} \mathbf{Q} \mathbf{B}^\top. \tag{11}
$$

**里程计噪声自适应**：利用 IMU 测量的水平加速度幅值 $a_{\text{mag}}$ 判断是否发生撞击。若 $a_{\text{mag}} \ge a_{\text{th}}$（如 $12\,\text{m/s}^2$），则将 $\mathbf{Q}$ 放大 $10\sim 100$ 倍，使滤波器迅速降低对里程计的信任，转而依赖视觉更新。

### 视觉更新

当视觉定位算法输出有效绝对位姿 $\mathbf{z}_v = \begin{bmatrix} x_v \\ y_v \\ \theta_v \end{bmatrix}$ 时，进入更新步骤。

#### 观测方程

视觉观测直接测量世界系位姿，故观测模型为  

$$
\mathbf{z}_v = \mathbf{X}_k + \delta\mathbf{X}_k + \mathbf{v},
$$

其中 $\mathbf{v} \sim \mathcal{N}(\mathbf{0}, \mathbf{R})$ 为观测噪声，$\mathbf{R}$ 可根据视觉质量（如可见标记点数量、距离）动态调整。观测矩阵为  

$$
\mathbf{H} = \mathbf{I}_{3\times 3}.
$$

#### 异常检测

计算新息（残差）  

$$
\mathbf{r} = \mathbf{z}_v \ominus \mathbf{X}_k = 
\begin{bmatrix}
x_v - x_k \\
y_v - y_k \\
\operatorname{wrap}(\theta_v - \theta_k)
\end{bmatrix},
$$

其中航向差被归化到 $[-\pi,\pi)$。若 $\|\mathbf{r}\|_{2}$ 超过预设阈值，再结合 IMU 数据判定：

- 若加速度幅值正常，判定为视觉瞬误（如标签误匹配），**丢弃本次观测**。
- 若检测到撞击（$a_{\text{mag}}$ 高），则认为里程计已不可靠，**接受该视觉观测**但可设置安全上限。

若视觉持续多帧一致（短时间窗口内视觉解自身方差小），亦视为可靠。

#### 卡尔曼更新

观测有效时，计算卡尔曼增益：

$$
\mathbf{K} = \mathbf{P}_k \mathbf{H}^\top (\mathbf{H} \mathbf{P}_k \mathbf{H}^\top + \mathbf{R})^{-1}. \tag{12}
$$

误差状态修正量  

$$
\delta\hat{\mathbf{X}} = \mathbf{K}\,\mathbf{r}. \tag{13}
$$

将修正注入名义状态（位姿复合）：

$$
\mathbf{X}_k \leftarrow \mathbf{X}_k \oplus \delta\hat{\mathbf{X}}.
$$

更新协方差：

$$
\mathbf{P}_k \leftarrow (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_k. \tag{14}
$$

#### 视觉失效

若视觉返回无效（$null$），则跳过更新，仅执行预测。此时协方差持续增长，表征不确定性增加；视觉恢复时，因 $\mathbf{P}$ 较大，卡尔曼增益会自动升高，滤波器能迅速重新收敛。

### 算法流程

1. **初始化**：设初始位姿 $\mathbf{X}_0$，协方差 $\mathbf{P}_0$。Pinpoint 自动清零，视觉系统启动。
2. **每帧执行**：
   - 读取 Pinpoint 增量 $\Delta\mathbf{u}$，读取 IMU 加速度 $a_{\text{mag}}$。
   - 动态调整 $\mathbf{Q}$：若 $a_{\text{mag}} \ge a_{\text{th}}$，增大 $\mathbf{Q}$。
   - 执行预测 (10)(11) 更新名义状态和协方差。
   - 若视觉有效：
     - 计算新息 $\mathbf{r}$，执行异常检测。
     - 若观测被接受，则执行更新 (12)-(14)。
   - 输出当前名义状态 $\mathbf{X}$ 作为定位结果。

该融合方案充分利用了 Pinpoint 高频估计的平滑性与视觉绝对观测的校正能力，并通过 IMU 辅助检测撞击，有效应对里程计失效与视觉跳变，实现了鲁棒的实时全场定位。