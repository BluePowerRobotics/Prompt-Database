# utility 工具库目录说明

`utility` 目录包含FTC机器人程序所需的各种数学工具、控制算法和数据处理类。

---

## 目录结构

| 文件/文件夹 | 功能说明 |
|-------------|----------|
| `Vector2D.java` | 二维向量类，提供平移、旋转、缩放、对称、点积等几何运算 |
| `Vector3D.java` | 三维向量类，提供绕轴旋转、平面投影、球坐标转换等空间运算 |
| `ActionRunner.java` | 动作执行器，基于 Road Runner `Action`，串行执行动作队列 |
| `HypParams.java` | 全局超参数配置类，使用 `@Config` 支持 FTC Dashboard 实时调参 |
| `Algebra/` | 代数工具目录，包含多项式方程求解器与复数运算 |
| `Geometry/` | 几何工具目录，包含凸多边形、欧拉角、投影器 |
| `PID/` | PID控制器目录，包含 PID、SVA、PIDSVA 控制器及 SlotConfig |
| `filter/` | 滤波器目录，包含均值滤波、角度滤波与 EMA 指数滤波 |
| `BST/` | 预留目录（当前仅有说明文件，无实现代码） |

---

## 核心类简介

### Vector2D
二维向量，FTC机器人坐标系的基础类型。提供 `getRadian`、`getDistance`、点积 `dot`、平移 `translate`、旋转 `rotate`、缩放 `scale`、中心/轴对称、中点 `getMidpoint`、极坐标转换 `fromPolar` 及角度归一化 `normalizeAngle` 等工具方法。

### Vector3D
三维向量，用于三维空间计算。提供点积 `dot`、叉积 `cross`、绕轴/绕坐标轴旋转 `rotateAroundAxis`/`rotateX/Y/Z`、平面投影 `projectToPlane`、平面对称 `symmetryAboutPlane`、球坐标转换 `fromSpherical` 等运算。

### ActionRunner
维护一个 Road Runner `Action` 的串行队列，通过在 OpMode 循环中调用 `update()` 逐个执行，动作完成后自动切换下一个。提供 `isBusy()` 判断是否仍有动作、`clear()` 清空队列。

### HypParams
全局超参数配置类，使用 `@Config` 注解支持 FTC Dashboard 实时调参。当前包含底盘最大速度 `maxV` 与最大角速度 `maxOmega`。