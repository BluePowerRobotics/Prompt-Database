# 三维空间点透视模型

设空间右手直角坐标系（世界坐标系）$W$ 的原点处固定一台理想的点透视摄像头（畸变不计）。摄像头初始姿态（零姿态）定义为：主光轴指向世界 $x$ 轴正方向，画面正上方与世界 $z$ 轴正方向重合。画面坐标系以图像中心为原点，向右为 $x_{p}$ 正方向，向上为 $y_{p}$ 正方向（单位：像素）。

已知摄像头相对于世界坐标系的姿态由一组内旋欧拉角 $\left( \theta_{p},\ \theta_{r},\ \theta_{y} \right)$ 描述，分别对应 **pitch**、**roll**、**yaw**，旋转顺序为 **ZYX**（先绕自身 $Z$ 轴转 $\theta_{y}$，再绕当前 $Y$ 轴转 $\theta_{p}$，最后绕当前 $X$ 轴转 $\theta_{r}$）。姿态角均为零时，相机坐标系与世界坐标系重合。

在摄像头的画面中，存在一物体。忽略物体自身的旋转，仅考虑其中心像素坐标 $\left( x_{p},\ y_{p} \right)$（单位：像素）以及大小。

下求物体中心在世界坐标系中的绝对坐标 $(x,y,z)$（单位：米）的封闭表达式。

为描述方便，定义缩放倍率 $k$（无量纲）为：

$$k = \frac{\text{物体当前像的某边长}}{\text{同一物体距摄像头 1 m 远处像的同一边长}}.$$

根据透视投影的相似关系

$$d = \frac{1}{k}.$$

先描述与相机的相对方向，定义相机坐标系C：固连在摄像机上，初始与 $W$ 重合。主光轴沿 $X_{c}$ 轴向前，水平向右对应 $- Y_{c}$ 方向，垂直向上对应 $Z_{c}$ 方向，则

$$X_{c} = d = \frac{1}{k}.$$

为便于推导，可另测参数 $l_{0}$：当长度为 1 m 的物体垂直于主光轴且距离摄像头 1 m 时，其在画面中的像长为 $l_{0}$ 像素。则在相机坐标系 $C$ 中，根据点透视，像素坐标与相机坐标的投影关系为

$$x_{p} = - l_{0}\frac{Y_{c}}{X_{c}},\quad\quad y_{p} = l_{0}\frac{Z_{c}}{X_{c}},$$

由此解得

$$Y_{c} = - \frac{X_{c}}{l_{0}}x_{p},\quad\quad Z_{c} = \frac{X_{c}}{l_{0}}y_{p}.$$

代入 $X_{c} = d$ ，不妨定义物平面尺度因子

$$A = \frac{d}{l_{0}} = \frac{1}{k\, l_{0}},$$

则

$$P_{c} = \begin{bmatrix}
X_{c} \\
Y_{c} \\
Z_{c}
\end{bmatrix} = \begin{bmatrix}
d \\
 - Ax_{p} \\
Ay_{p}
\end{bmatrix}.$$

下面按内旋 ZYX 顺序从世界坐标系旋转到相机坐标系，从相机坐标系到世界坐标系的变换为

$$P_{w} = R_{z}\left( \theta_{y} \right)\, R_{y}\left( \theta_{p} \right)\, R_{x}\left( \theta_{r} \right)\, P_{c}.$$

其中

$$R_{x}(\theta) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\theta & - \sin\theta \\
0 & \sin\theta & \cos\theta
\end{bmatrix},\ R_{y}(\theta) = \begin{bmatrix}
\cos\theta & 0 & \sin\theta \\
0 & 1 & 0 \\
 - \sin\theta & 0 & \cos\theta
\end{bmatrix},\ R_{z}(\theta) = \begin{bmatrix}
\cos\theta & - \sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}$$

逐级计算：

1. **绕 $X$ 轴转 $\theta_{r}$（roll）**

$$\begin{matrix}
v_{1x} & = d, \\
v_{1y} & = Y_{c}\cos\theta_{r} - Z_{c}\sin\theta_{r} = - A\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right), \\
v_{1z} & = Y_{c}\sin\theta_{r} + Z_{c}\cos\theta_{r} = A\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right).
\end{matrix}$$

2. **绕 $Y$ 轴转 $\theta_{p}$（pitch）**

$$\begin{matrix}
v_{2x} & = v_{1x}\cos\theta_{p} + v_{1z}\sin\theta_{p} = d\cos\theta_{p} + A\sin\theta_{p}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right), \\
v_{2y} & = v_{1y} = - A\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right), \\
v_{2z} & = - v_{1x}\sin\theta_{p} + v_{1z}\cos\theta_{p} = - d\sin\theta_{p} + A\cos\theta_{p}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right).
\end{matrix}$$

3. **绕 $Z$ 轴转 $\theta_{y}$（yaw）**

$$\begin{matrix}
x & = v_{2x}\cos\theta_{y} - v_{2y}\sin\theta_{y} \\
 & = d\cos\theta_{p}\cos\theta_{y} + A\sin\theta_{p}\cos\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) + A\sin\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right), \\
y & = v_{2x}\sin\theta_{y} + v_{2y}\cos\theta_{y} \\
 & = d\cos\theta_{p}\sin\theta_{y} + A\sin\theta_{p}\sin\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) - A\cos\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right), \\
z & = v_{2z} = - d\sin\theta_{p} + A\cos\theta_{p}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right).
\end{matrix}$$

将 $d = \frac{1}{k}$，$A = \frac{1}{k\, l_{0}}$ 代入式 (4)，即得完整表达式：

$$\boxed{\begin{matrix}
x & = \frac{1}{k}\cos\theta_{p}\cos\theta_{y} + \frac{1}{k\, l_{0}}\left\lbrack \sin\theta_{p}\cos\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) + \sin\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right) \right\rbrack, \\
y & = \frac{1}{k}\cos\theta_{p}\sin\theta_{y} + \frac{1}{k\, l_{0}}\left\lbrack \sin\theta_{p}\sin\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) - \cos\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right) \right\rbrack, \\
z & = - \frac{1}{k}\sin\theta_{p} + \frac{1}{k\, l_{0}}\cos\theta_{p}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right).
\end{matrix}}$$

特别地，当 $\theta_{p} = \pm \frac{\pi}{2}$ 时，$\cos\theta_{p} = 0,\ \sin\theta_{p} = \pm 1$，式 (4) 简化为

$$\boxed{\begin{matrix}
x & = \frac{1}{k\, l_{0}}\left\lbrack \pm \cos\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) + \sin\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right) \right\rbrack, \\
y & = \frac{1}{k\, l_{0}}\left\lbrack \pm \sin\theta_{y}\left( - x_{p}\sin\theta_{r} + y_{p}\cos\theta_{r} \right) - \cos\theta_{y}\left( x_{p}\cos\theta_{r} + y_{p}\sin\theta_{r} \right) \right\rbrack, \\
z & = - \frac{1}{k}( \pm 1) = \mp \frac{1}{k}.
\end{matrix}}$$

该表达式在极值姿态下仍然可直接计算，不产生奇异，无需额外的条件分支。