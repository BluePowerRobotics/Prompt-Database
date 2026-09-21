# Geometry 几何计算工具

## 概述

本目录包含几何计算相关的工具类，主要用于凸多边形、欧拉角和三维投影等几何运算。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `ConvexPolygon.java` | 凸多边形类，支持碰撞检测、坐标转换、交集计算等 |
| `EulerAngle.java` | 欧拉角类，封装 roll/pitch/yaw 并实现三维旋转变换 |
| `Projector.java` | 投影器类，将相机图像坐标投影到三维空间坐标 |

## ConvexPolygon 使用说明

### 功能特性

- 凸多边形的创建和验证
- 坐标转换（相对坐标↔绝对坐标）
- 点是否在多边形内的检测
- 多边形碰撞检测（包含、相交）
- 多边形交集计算
- 最近点计算

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

// 创建三角形
Vector2D v1 = new Vector2D(0, 0);
        Vector2D v2 = new Vector2D(10, 0);
        Vector2D v3 = new Vector2D(5, 10);
        ConvexPolygon triangle = new ConvexPolygon(v1, v2, v3);

        // 使用数组创建多边形（任意顶点数）
        Vector2D[] verts = { v1, v2, v3 };
        ConvexPolygon poly = new ConvexPolygon(verts);

        // 检测点是否在多边形内
        Vector2D point = new Vector2D(5, 5);
        boolean contains = triangle.Contains(point);

        // 检测两个多边形是否相交
        boolean intersects = triangle.IsIntersected(poly);

        // 计算最近点向量（从点到多边形边界）
        Vector2D nearest = triangle.NearestVectorFrom(25, 10);

        // 坐标转换：相对坐标转绝对坐标
        ConvexPolygon absolutePolygon = triangle.inAbsolute(100, 200, Math.toRadians(45));
```

### 方法签名

| 方法 | 说明 | 参数 |
|------|------|------|
| `ConvexPolygon(Vector2D[] vertices)` | 构造函数，接受任意数量（≥3）顶点数组 | `Vector2D[]` |
| `ConvexPolygon(p1...pn)` | 便捷构造函数，支持3-6个顶点 | `Vector2D...`（3-6个） |
| `Contains(x, y)` | 检测点是否在多边形内 | `double x, double y` |
| `Contains(Vector2D)` | 检测点是否在多边形内 | `Vector2D point` |
| `Contains(ConvexPolygon)` | 检测多边形是否完全包含另一个 | `ConvexPolygon other` |
| `IsIntersected(ConvexPolygon)` | 检测两个多边形是否相交 | `ConvexPolygon other` |
| `IntersectWith(ConvexPolygon)` | 计算两个多边形的交集 | `ConvexPolygon clip` |
| `NearestVectorFrom(x, y)` / `NearestVectorFrom(Vector2D)` | 计算点到多边形边界的最近向量 | `double x, double y` / `Vector2D` |
| `inRelative(x, y, theta)` / `inRelative(Pose2d)` | 将多边形转换为相对坐标 | `double x, double y, double theta` / `Pose2d` |
| `inAbsolute(x, y, theta)` / `inAbsolute(Pose2d)` | 将多边形转换为绝对坐标 | `double x, double y, double theta` / `Pose2d` |
| `getVertices()` / `getVertex(int)` / `getVertexCount()` | 获取顶点信息 | — |

### 注意事项

1. 构造函数会自动验证顶点是否构成凸多边形，非凸多边形会抛出异常
2. 顶点会自动按逆时针方向排序
3. 交集计算要求两个多边形必须相交，否则抛出异常
4. `Pose2d` 来自 Road Runner 库，使用 `inRelative(Pose2d)` / `inAbsolute(Pose2d)` 需导入该库

## EulerAngle 使用说明

### 功能特性

- 封装 roll/pitch/yaw 三个旋转角
- `reform(Vector3D)`：将向量按 yaw → pitch → roll 顺序进行旋转
- `transform(Vector3D)`：与 `reform` 相反方向的三维旋转变换

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Geometry.EulerAngle;
import org.firstinspires.ftc.teamcode.utility.Vector3D;

EulerAngle pose = new EulerAngle(roll, pitch, yaw);

Vector3D src = new Vector3D(1, 2, 3);
Vector3D rotated = pose.reform(src);    // 正向旋转
Vector3D back = pose.transform(src);    // 反向旋转

double yaw = pose.getYaw();
pose.setYaw(Math.toRadians(30));
```

### 方法签名

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `EulerAngle(roll, pitch, yaw)` | 构造函数 | `EulerAngle` |
| `getRoll()/setRoll(...)` 等 | roll/pitch/yaw 的 getter/setter | `double` |
| `reform(Vector3D vec)` | 正向三维旋转变换 | `Vector3D` |
| `transform(Vector3D vec)` | 反向三维旋转变换 | `Vector3D` |

## Projector 使用说明

### 功能特性

- 组合 `EulerAngle` 姿态与焦距 `l0`
- `project(x_p, y_p, r)`：将相机图像坐标 `(x_p, y_p)` 及半径 `r` 投影到三维空间坐标

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Geometry.EulerAngle;
import org.firstinspires.ftc.teamcode.utility.Geometry.Projector;
import org.firstinspires.ftc.teamcode.utility.Vector3D;

Projector projector = new Projector(new EulerAngle(roll, pitch, yaw), l0);

// 将相机图像坐标投影到三维空间
Vector3D world = projector.project(x_p, y_p, r);

// 运行时更新姿态或焦距
projector.setPose(newPose);
projector.setL0(newL0);
```

### 方法签名

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `Projector(EulerAngle pose, double l0)` | 构造函数 | `Projector` |
| `getPose()/setPose(...)` | 姿态 getter/setter | `EulerAngle` |
| `getL0()/setL0(...)` | 焦距 getter/setter | `double` |
| `project(x_p, y_p, r)` | 图像坐标投影到三维空间 | `Vector3D` |