## 三维点几何工具类实现规范 (LLM Prompt)

你是一位软件开发工程师，需要实现一个**三维点类 `Point3D`**，用于表示和操作三维空间中的点。该类提供丰富的**静态几何变换方法**、**实例属性访问**和**向量运算**，可作为机器人定位、路径规划、计算机视觉等模块的基础数学工具。

---

### 1. 类职责与概述

- 表示三维笛卡尔坐标系中的一个点/向量，包含 `x`、`y`、`z` 坐标。
- 提供实例方法获取球坐标（方位角、极角、距离）。
- 提供实例方法进行加法、范围限制（长方体与球体）和零点判断。
- 提供大量静态方法：距离、点积、叉积、归一化、平移、旋转、缩放、对称、投影、平面相关计算等。
- 提供零容差常量，用于浮点数比较和检测退化情况。
- 支持与 `Point2D` 的转换（投影到平面）。

---

### 2. 内部状态与构造

- **字段**：
  - `private double x, y, z` – 坐标。
  - 提供 `getX()`, `setX(double)`, `getY()`, `setY(double)`, `getZ()`, `setZ(double)`。
- **构造函数**：
  - `Point3D(double x, double y, double z)`
  - `Point3D(Point3D other)` – 拷贝构造。
- **实例方法**：
  - `double getDistance()` – 返回原点到该点的距离 `√(x² + y² + z²)`。
  - `double getAzimuth()` – 返回方位角 `atan2(y, x)`。
  - `double getPolar()` – 返回极角 `acos(z / distance)`，若距离几乎为 0 则返回 0。
  - `void add(Point3D other)` – 逐分量相加（修改自身）。
  - `void clamp(Point3D min, Point3D max)` – 将坐标限制在长方体范围内。
  - `void clamp(double maxDistance, Point3D p)` – 如果点到 `p` 的距离超过 `maxDistance`，将其沿连线移回球面。
  - `boolean isZero()` – 判断三个分量是否都在 `zeroTolerance` 内。
- **字符串表示**： `toString()` 返回 `"( x , y , z )"` 格式。

---

### 3. 类常量

- `Point3D ZERO = new Point3D(0, 0, 0)` – 原点常量。
- `double zeroTolerance = 1e-10` – 用于相等性判断和退化情况处理的容差。

---

### 4. 基本静态方法

#### 4.1 相等性判断
```java
public static boolean equal(Point3D p1, Point3D p2)
```
- 若 `|x1-x2| < zeroTolerance` 且 `|y1-y2| < zeroTolerance` 且 `|z1-z2| < zeroTolerance`，返回 `true`。

#### 4.2 距离计算
```java
public static double distance(Point3D p1, Point3D p2)
```
- 返回欧几里得距离 `√((x1-x2)² + (y1-y2)² + (z1-z2)²)`。

#### 4.3 向量运算

- **点积** `dot(Point3D p1, Point3D p2)`：`x1*x2 + y1*y2 + z1*z2`。
- **叉积** `cross(Point3D p1, Point3D p2)`：返回 `(y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2)`。
- **模长** `magnitude(Point3D p)`：`√(x² + y² + z²)`。
- **归一化** `normalize(Point3D p)`：若模长 < `zeroTolerance` 返回 `ZERO`，否则返回各分量除以模长。

---

### 5. 平移变换

- `translate(Point3D p, Point3D offset)` – 按另一个点平移。
- `translateXYZ(Point3D p, double dx, double dy, double dz)` – 按分量平移。

---

### 6. 旋转变换

- `rotateX(Point3D p, double angle)` – 绕 X 轴旋转（保持 x 不变，y 和 z 按二维旋转）。
  ```
  y' = y*cos - z*sin
  z' = y*sin + z*cos
  ```
- `rotateY(Point3D p, double angle)` – 绕 Y 轴旋转。
  ```
  x' = x*cos + z*sin
  z' = -x*sin + z*cos
  ```
- `rotateZ(Point3D p, double angle)` – 绕 Z 轴旋转（同二维旋转在 XY 平面）。
  ```
  x' = x*cos - y*sin
  y' = x*sin + y*cos
  z' = z
  ```
- `rotateAroundAxis(Point3D p, Point3D axis, double angle)` – 绕任意轴（罗德里格斯公式）：
  - 归一化 `axis`。
  - 计算公式：`p' = p*cosθ + (axis × p)*sinθ + axis*(axis·p)*(1-cosθ)`。

---

### 7. 缩放变换

- `scale(Point3D p, double factor)` – 绕原点缩放。
- `scale(Point3D p, double factor, Point3D center)` – 绕指定中心缩放（先平移到中心，缩放，再平移回来）。

---

### 8. 对称变换

#### 8.1 中心对称
- `centralSymmetry(Point3D p)` – 关于原点的对称：`(-x, -y, -z)`。
- `centralSymmetry(Point3D p, Point3D center)` – 关于中心点的对称：`(2*center.x - x, 2*center.y - y, 2*center.z - z)`。

#### 8.2 坐标平面对称
- `symmetryAboutXYPlane(Point3D p)` – 返回 `(x, y, -z)`。
- `symmetryAboutYZPlane(Point3D p)` – 返回 `(-x, y, z)`。
- `symmetryAboutXZPlane(Point3D p)` – 返回 `(x, -y, z)`。

#### 8.3 任意平面对称
- `symmetryAboutPlane(Point3D point, Point3D planeNormal, Point3D planePoint)`
  - 先计算点在平面上的投影 `projection`。
  - 然后计算投影点关于原始点的中心对称点返回。

---

### 9. 平面与点的计算

#### 9.1 点到平面的距离
`distanceToPlane(Point3D point, Point3D planeNormal, Point3D planePoint)`
- 计算点到平面的有符号距离：`dot( point - planePoint, planeNormal )`，返回绝对值。

#### 9.2 点在平面上的投影
`projectToPlane(Point3D point, Point3D planeNormal, Point3D planePoint)`
- 计算向量 `diff = point - planePoint`。
- 投影点 = `point - planeNormal * dot(diff, planeNormal)`。

#### 9.3 三点计算平面法向量
`calculatePlaneNormal(Point3D p1, Point3D p2, Point3D p3)`
- 向量 `v1 = p2 - p1`, `v2 = p3 - p1`。
- 返回 `normalize(cross(v1, v2))`。

#### 9.4 直线与平面交点（直线经过原点）
`linePlaneIntersection(Point3D p, Point3D planeNormal, Point3D planePoint)`
- 直线参数方程：`X = t * p`，平面方程：`n·(X - P0) = 0`。
- 若 `p` 为零向量或直线与平面平行（`|n·p| < 1e-9`），返回 `null`。
- 解参数 `t = - (n·P0) / (n·p)`，交点 = `scale(p, t)`。

---

### 10. 球坐标与坐标转换

- `fromSpherical(double azimuth, double polar, double distance)` – 从球坐标构造点：
  ```
  x = distance * sin(polar) * cos(azimuth)
  y = distance * sin(polar) * sin(azimuth)
  z = distance * cos(polar)
  ```

- **转换为二维点**：
  `toPoint2D(Point3D p, Point3D planeNormal, Point3D planePoint)`
  - 先将点投影到平面，然后取投影点的 `(x, y)` 作为二维坐标（假设平面平行于 XY 平面或用于简化）。更通用的做法应使用 `Point2D.toPoint3D` 的逆过程，但此处返回简单的 `(x, y)`。

---

### 11. 其他方法

- `midpoint(Point3D p1, Point3D p2)` – 返回中点 `((x1+x2)/2, (y1+y2)/2, (z1+z2)/2)`。
- `scale(Point3D p, double factor)` / `scale(Point3D p, double factor, Point3D center)` – 缩放。

---

### 12. 使用示例

```java
Point3D p1 = new Point3D(1, 2, 3);
Point3D p2 = new Point3D(4, 5, 6);

double dist = Point3D.distance(p1, p2);
double dot = Point3D.dot(p1, p2);
Point3D cross = Point3D.cross(p1, p2);

Point3D unit = Point3D.normalize(p1);
Point3D rotated = Point3D.rotateZ(p1, Math.PI/2);

// 投影到平面
Point3D planeNormal = new Point3D(0, 0, 1);
Point3D planePoint = new Point3D(0, 0, 0);
Point3D proj = Point3D.projectToPlane(p1, planeNormal, planePoint);

// 线面交点（从原点沿 p1 方向射线与平面交点）
Point3D intersection = Point3D.linePlaneIntersection(p1, planeNormal, planePoint);
```

---

### 13. 实现注意事项

- 使用 `double` 类型保证精度，注意处理 `NaN` 和无穷大值（尽管当前未显式检测，但调用方需注意）。
- 归一化、球坐标方法需处理零向量的退化情况。
- 直线与平面交点、投影等方法假设法向量不一定为单位向量，但内部计算需兼容。
- 静态方法不修改传入对象，始终返回新实例（除实例方法 `add`、`clamp` 外）。
- 包含必要的 Javadoc 注释，特别是复杂公式（如罗德里格斯公式）需注明。
- 与 `Point2D` 配合使用时，注意坐标系的转换逻辑。

---

请根据以上规范，生成完整的 `Point3D` 工具类，确保代码清晰、高性能且附有详细文档注释。