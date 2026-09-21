# Utility 工具库目录说明

## 概述

本目录包含 FTC 机器人项目中常用的工具类和算法库，为机器人控制提供数学计算、滤波处理、轨迹模拟等基础功能支持。

## 目录结构

```
utility/
├── ActionRunner.java      # Action队列执行器
├── Vector2D.java          # 2D向量类
├── Vector3D.java          # 3D向量类
├── Algebra/               # 代数计算工具
│   └── EquationSolver.java
├── Geometry/              # 几何计算工具
│   └── ConvexPolygon.java
├── RK4/                   # 四阶龙格-库塔弹道模拟
│   ├── AutoSelect.java
│   ├── CalibrationHelper.java
│   ├── ParameterCalculator.java
│   ├── ParameterCalibrationApp.java
│   ├── ProjectileDynamics.java
│   ├── ProjectileParameters.java
│   ├── ProjectileState.java
│   ├── RobotState.java
│   ├── Solver.java
│   ├── TargetPredictor.java
│   ├── TrajectorySimulator.java
│   └── TestRK4.java
└── filter/                # 滤波算法
    ├── MeanFilter.java
    ├── AngleMeanFilter.java
    ├── AngleWeightedMeanFilter.java
    └── kalmanfilter/
        ├── OneDimensionKalmanFilter.java
        ├── PosVelTuple.java
        └── jama/          # JAMA矩阵库
```

## 文件功能说明

### 根目录文件

| 文件 | 功能说明 |
|------|----------|
| `ActionRunner.java` | Road Runner Action队列管理器，支持串行执行多个Action，自动处理Action切换和仪表盘显示 |
| `Vector2D.java` | 2D向量类，支持坐标转换、旋转、平移、缩放、点积、叉积等运算 |
| `Vector3D.java` | 3D向量类，支持3D空间中的向量运算、坐标变换、平面投影等 |

### 子目录说明

| 目录 | 功能说明 |
|------|----------|
| `Algebra/` | 代数计算工具，包含方程求解器 |
| `Geometry/` | 几何计算工具，包含凸多边形处理 |
| `RK4/` | 四阶龙格-库塔数值积分法实现的弹道模拟系统 |
| `filter/` | 数据滤波算法，包含均值滤波、角度滤波和卡尔曼滤波 |

### 使用建议

1. **向量运算**：使用 `Vector2D` 和 `Vector3D` 处理坐标计算
2. **动作管理**：使用 `ActionRunner` 管理复杂的自动序列
3. **数据滤波**：根据需求选择 `MeanFilter` 或卡尔曼滤波
4. **弹道计算**：使用 `RK4/` 目录下的类进行抛射物轨迹模拟和参数求解

## 子目录指南

各子目录下均有独立的 `Guide.md` 文件，详细说明该目录内文件的使用方法：

- [Algebra/Guide.md](./Algebra/Guide.md)
- [Geometry/Guide.md](./Geometry/Guide.md)
- [RK4/Guide.md](./RK4/Guide.md)
- [filter/Guide.md](./filter/Guide.md)