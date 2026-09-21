# 单地图位姿解算

## 一、坐标系与已知条件

- **场地坐标系 {W}**：原点在场地中心，x 向右，y 向前，z 向上。
- **蜂巢枢轴**：轴线沿 y 轴，过点 $p_{\text{pivot}} = (0,0,h)$，单位方向向量 $\mathbf{u} = (0,1,0)$。
- **地图**：存储的是蜂巢处于**水平参考状态**（记为 $\theta=0$）时，本方 8 个 AprilTag 的位姿。
- **实际蜂巢**：相对于水平状态绕轴旋转了角度 $\theta$。
- **Limelight 返回**：使用水平地图解算出的机器人位姿记为 $T' = \begin{bmatrix} R' & p' \ 0 & 1 \end{bmatrix} $，其中 $p' = (x', y', z')^T$。
- **真实位姿**：机器人实际位姿记为 $T = \begin{bmatrix} R & p \ 0 & 1 \end{bmatrix} $，其中 $p = (x, y, z)^T$。

***

## 二、错误位姿与真实位姿的关系

绕 y 轴旋转角度 $\phi$ 的齐次变换为：

$$
T\_{\text{rot}}(\phi) =
\begin{bmatrix}
\cos\phi & 0 & \sin\phi & -h\sin\phi \\
0 & 1 & 0 & 0 \\
-\sin\phi & 0 & \cos\phi & h(1-\cos\phi) \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

推导：旋转矩阵 $ R\_y(\phi) = \begin{bmatrix} \cos\phi & 0 & \sin\phi \ 0 & 1 & 0 \ -\sin\phi & 0 & \cos\phi \end{bmatrix} $，平移部分为 $ (I - R\_y(\phi)) p\_{\text{pivot}} = (-h\sin\phi, 0, h(1-\cos\phi))^T $。

根据之前的一般推导，错误位姿与真实位姿满足：
$$
T' = T\_{\text{rot}}(\phi) , T
$$
其中 $ \phi = -\theta $（$ \theta $ 为蜂巢实际旋转角）。也就是说，Limelight 返回的 botpose 相当于真实位姿先绕枢轴旋转了 $ \phi = -\theta $ 后的结果。

反过来：
$$
T = T\_{\text{rot}}(-\phi) , T'
$$

***

## 三、位置修正公式

由 $ T' = T\_{\text{rot}}(\phi) T $ 可得：
$$
p' = R\_y(\phi) , p + (-h\sin\phi, 0, h(1-\cos\phi))^T
$$

展开成分量：
$$
\begin{aligned}
x' &= x \cos\phi + z \sin\phi - h \sin\phi \\
y' &= y \\
z' &= -x \sin\phi + z \cos\phi + h(1-\cos\phi)
\end{aligned}
$$

反过来，由 $ p' $ 求真实位置 $ p $：
$$
p = R\_y(-\phi) \left( p' - (-h\sin\phi, 0, h(1-\cos\phi))^T \right)
$$
即：
$$
\begin{aligned}
x &= x' \cos\phi - z' \sin\phi + h \sin\phi \\
y &= y' \\
z &= x' \sin\phi + z' \cos\phi + h(1-\cos\phi)
\end{aligned}
$$

***

## 四、利用机器人物理约束解算 $\phi $

机器人在地面上运动，真实位置高度 $z \approx 0$。令 $z = 0$：
$$
x' \sin\phi + z' \cos\phi + h(1-\cos\phi) = 0
$$
整理：
$$
x' \sin\phi + (z' - h) \cos\phi + h = 0
$$

令 $A = x' $，$B = z' - h$，$C = -h$，则：
$$
A \sin\phi + B \cos\phi = C
$$
这是一个标准三角函数方程，其**两族解**为：
$$
\phi_1 = \alpha - \operatorname{atan2}(B, A),\qquad
\phi_2 = \pi - \alpha - \operatorname{atan2}(B, A),\qquad \alpha = \arcsin\!\left(\frac{C}{\sqrt{A^2+B^2}}\right)
$$
两族解恒相差 $\pi - 2\alpha$，故一般情况下只有一族落在合理区间内。HIVE 最大仰角约 $30^\circ$，
据此按 $|\phi| \le$ `hiveCellUpAngleDeg` 筛选；若两族都在窗口内则取 $|\phi|$ 较小者。
（可解性前提：$|C| \le \sqrt{A^2+B^2}$。）

> **退化条件**：由解算模型 $F(\phi) = x'\sin\phi + (z'-h)\cos\phi + h$ 恒有 $F(0) = z'$，
> 因此 $|z'| \approx 0$ 时位置通路的解必然退化到 $\phi = 0$（$\theta = 0$），与 HIVE 真实倾角无关。
> 判别方法见 [MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.md) §3.2-3（姿态通路交叉校验）。

解出 $\phi$ 后，代入位置修正公式得到真实位置。

***

## 五、姿态修正公式

由 ( R' = R\_y(\phi) R )，得：
$$
R = R\_y(-\phi) R'
$$
其中 $ R\_y(-\phi) = \begin{bmatrix} \cos\phi & 0 & -\sin\phi \ 0 & 1 & 0 \ \sin\phi & 0 & \cos\phi \end{bmatrix} $。

若只需要二维位姿 $ (x, y, \text{yaw}) $，可由修正后的旋转矩阵提取 yaw。
$R'$ 的第一列即机器人 $x$ 轴在场坐标系下的方向（SDK 约定：内旋 yaw → pitch → roll，
故 $R_{00} = \cos\psi'\cos P'$，$R_{10} = \sin\psi'\cos P'$，$R_{20} = -\sin P'$）：
$$
\text{yaw} = \operatorname{atan2}\big(R_{10},\ \cos\phi\,R_{00} - \sin\phi\,R_{20}\big)
$$
> 早期本文档写作 $\text{yaw} \approx \operatorname{atan2}(R_{21}, R_{11})$，该式在机器人水平时恒为 0，属笔误。

***

## 六、总结流程

1. 从 Limelight 获取 botpose $ (x', y', z') $ 和姿态 $ R' $。
2. 解方程 $ x' \sin\phi + (z' - h) \cos\phi + h = 0 $，得到 $\phi$（即 $ -\theta $）。
3. 计算真实位置：
   $$
   \begin{aligned}
   x &= x' \cos\phi - z' \sin\phi + h \sin\phi \\
   y &= y' \\
   z &= x' \sin\phi + z' \cos\phi + h(1-\cos\phi)
   \end{aligned}
   $$
   注意：对解析根而言 $z \equiv 0$ 是**恒等式**（$z = C + h = 0$），不能作为质量校验量，
   需改用姿态通路交叉校验（见 [MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.md) §3.2-3）。
4. 计算真实姿态：$ R = R\_y(-\phi) R' $，按 §五 的公式提取 $\text{yaw}$。
5. 将修正后的 $ (x, y, \text{yaw}) $ 送入 EKF 或直接使用。

这样，**只需一个水平状态的地图**，就能适应蜂巢任意角度的旋转，无需动态上传地图。注意：必须只使用**本方蜂巢的标签**，避免红蓝蜂巢混合导致旋转轴不一致。

**前提条件（实机确认）：Limelight 的 `snap to ground` 必须设为 `No`。** 该选项开启时，
Limelight 会把解算位姿强行压到地面平面上（等价于 $z' \equiv 0$），
位置通路的倾角信息被完全抹掉，本方案会恒输出 $\theta = 0$。
