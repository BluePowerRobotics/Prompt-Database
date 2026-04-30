## 二维点几何工具类实现规范 (LLM Prompt)

你是一位软件开发工程师，需要实现一个**二维点类 `Point2D`**，用于表示和操作二维平面上的点。该类提供丰富的**静态几何变换方法**和**实例属性访问**，可作为机器人定位、路径规划、计算机视觉等模块的基础数学工具。

---

### 1. 类职责与概述

- 表示二维笛卡尔坐标系中的一个点，包含 `x` 和 `y` 坐标。
- 提供实例方法获取极坐标（弧度、距离）。
- 提供大量静态方法：平移、旋转、缩放、对称、坐标转换、中点计算、点积叉积、极坐标构造等。
- 提供零容差常量，用于浮点数比较。
- 支持向三维空间的映射（结合 `Point3D` 和平面法向量）。

---

### 2. 内部状态与构造

- **字段**：
  - `private double x, y` – 坐标。
  - 提供 `getX()`, `setX(double)`, `getY()`, `setY(double)`。
- **构造函数**：
  - `Point2D(double x, double y)`
  - `Point2D(Point2D p)` – 拷贝构造。
- **实例方法**：
  - `double getRadian()` – 返回极角，使用 `Math.atan2(y, x)`。
  - `double getDistance()` – 返回到原点的距离，使用 `Math.hypot(x, y)`。
- **字符串表示**： `toString()` 返回 `"( x , y )"` 格式。

---

### 3. 类常量

- `Point2D ZERO = new Point2D(0, 0)` – 原点常量。
- `double zeroTolerance = 1e-10` – 用于相等性判断和退化情况处理的容差。

---

### 4. 基本静态方法

#### 4.1 相等性判断
```java
public static boolean equal(Point2D p1, Point2D p2)
```
- 若 `|x1-x2| < zeroTolerance` 且 `|y1-y2| < zeroTolerance`，返回 `true`。

#### 4.2 距离计算
```java
public static double getDistance(Point2D p1, Point2D p2)
```
- 返回欧几里得距离 `√((x1-x2)² + (y1-y2)²)`。

#### 4.3 中点计算
```java
public static Point2D getMidpoint(Point2D p1, Point2D p2)
public static Point2D getMidpoint(double x1, double y1, double x2, double y2)
```
- 返回 `((x1+x2)/2, (y1+y2)/2)`。

---

### 5. 平移变换

- `translate(Point2D p, Point2D offset)` – 按另一个点偏移。
- `translate(Point2D p, double dx, double dy)` – 按分量偏移。
- `translateRD(Point2D p, double radian, double distance)` – 沿指定**弧度方向**移动指定**距离**。实现：`x + distance*cos(radian)`, `y + distance*sin(radian)`。

---

### 6. 旋转变换

- `rotate(Point2D p, double radian)` – 绕原点旋转 `radian` 弧度。标准公式：
  ```
  x' = x*cos(θ) - y*sin(θ)
  y' = x*sin(θ) + y*cos(θ)
  ```
- `rotate(Point2D p, double radian, Point2D center)` – 绕指定中心点旋转。通过平移到原点、旋转、平移回来实现。

---

### 7. 缩放变换

- `scale(Point2D p, double factor)` – 绕原点缩放。
- `scale(Point2D p, double factor, Point2D center)` – 绕指定中心缩放。实现类似旋转：先平移到中心，缩放，再平移回来。

---

### 8. 对称变换

#### 8.1 中心对称
- `getCentralSymmetry(Point2D p)` – 关于原点的对称：`(-x, -y)`。
- `getCentralSymmetry(Point2D p, Point2D center)` – 关于中心点的对称：`(2*center.x - x, 2*center.y - y)`。

#### 8.2 轴对称（关于任意直线 y = kx + b）
- `getAxisSymmetry(Point2D p, double k, double b)` – 主入口，处理三种情况：
  - **斜率无穷大**（垂直直线 x = b）：调用 `getAxisSymmetryVertical`。
  - **斜率 ≈ 0**（水平直线 y = b）：调用 `getAxisSymmetryHorizontal`。
  - **普通斜线**：调用 `getAxisSymmetrySlant`。
- **垂直直线对称** `getAxisSymmetryVertical(p, x)`：返回 `(2*x - p.x, p.y)`。
- **水平直线对称** `getAxisSymmetryHorizontal(p, y)`：返回 `(p.x, 2*y - p.y)`。
- **斜线对称** `getAxisSymmetrySlant(p, k, b)`：
  - 使用公式：
    ```
    denominator = 1 + k²
    x' = ((1 - k²)*x + 2*k*(y - b)) / denominator
    y' = (2*k*x + (k² - 1)*y + 2*b) / denominator
    ```

---

### 9. 极坐标与笛卡尔坐标转换

- `fromPolar(double radian, double distance)` – 从极坐标创建笛卡尔点：`(distance*cos(radian), distance*sin(radian))`。

---

### 10. 向量运算（点积与叉积）

- `dot(Point2D p1, Point2D p2)` – 返回 `x1*x2 + y1*y2`。
- `cross(Point2D p1, Point2D p2)` – 返回一个新的 `Point2D`，其 `x = x1*y2 - y1*x2`，`y = y1*x2 - x1*y2`（可视为标量叉积在两个维度上的分量表示）。

---

### 11. 三维映射

- `toPoint3D(Point2D p, Point3D planeNormal, Point3D planePoint)`：
  - 将二维点映射到三维空间中由法向量 `planeNormal` 和平面上的基准点 `planePoint` 定义的平面上。
  - 实现步骤：
    1. 归一化法向量 `unitNormal = normalize(planeNormal)`。
    2. 寻找平面上两个正交基向量 `U` 和 `V`：
       - `U` 为与法向量正交的单位向量（通过 Gram-Schmidt 过程：选择一个参考向量（如 X 轴），若它与法向量平行则选 Y 轴，然后投影得到 `U`）。
       - `V = cross(unitNormal, U)`（右手系）。
    3. 将二维点 `(x, y)` 解释为沿 `U` 和 `V` 的分量：`offset = x*U + y*V`。
    4. 最终三维点 = `planePoint + offset`。
- 辅助方法 `findUAxis(Point3D normal)` 用于计算 `U` 轴。

---

### 12. 使用示例

```java
Point2D p = new Point2D(3, 4);
System.out.println(p.getDistance()); // 5.0
System.out.println(p.getRadian());   // ~0.927 rad (53°)

Point2D rotated = Point2D.rotate(p, Math.PI/2); // (-4, 3)
Point2D moved = Point2D.translateRD(p, Math.PI/4, Math.sqrt(2)); // 沿45°移动1.414

// 关于直线 y = 0.5x + 1 的对称点
Point2D sym = Point2D.getAxisSymmetry(p, 0.5, 1.0);

// 向量点积
double dot = Point2D.dot(new Point2D(1,0), new Point2D(0,1)); // 0
```

---

### 13. 实现注意事项

- 使用 `double` 类型保证精度。
- 轴对称方法需处理垂直线和水平线退化情况。
- 三维映射依赖于 `Point3D` 类（需单独实现），但此处仅给出接口逻辑。
- 静态方法不修改传入的 `Point2D` 对象，始终返回新实例（不可变性）。
- 代码注释应详细，尤其是对称和投影公式。
- 可考虑实现 `equals()` 和 `hashCode()` 以便用于集合，但非强制。

---

请根据以上规范，生成完整的 `Point2D` 工具类，确保代码清晰、安全且附有 Javadoc 文档。