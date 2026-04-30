## 数学工具类实现规范 (LLM Prompt)

你是一位计算机视觉/机器人软件工程师。请实现一个**数学工具类 `MathSolver`**，集成了多项式方程求根（1~4次）、最小二乘直线拟合、角度归一化、坐标转换以及单位换算等常用功能。该类用于 FTC 机器人控制、传感器标定和路径规划等场景。

---

### 1. 功能概述

- 求解一元一至四次方程的实数根（包含多重根与判别式逻辑）。
- 使用最小二乘法拟合二维点集到一条直线，返回 `Line` 对象。
- 角度的归一化处理（限制到 `[-π, π]` 范围且处理 NaN）。
- 坐标转换：自定义的 `Point2D` 与 RoadRunner `Pose2d` 之间互相转换（含旋转 -90° 或 90° 的适配）。
- 数值工具：高精度 `hypot`（多参数）、平均值计算、度量单位转换（毫米↔英寸）。
- 符号函数 `sgn`。

---

### 2. 多项式方程求根

#### 2.1 一元一次方程 `solve1(a, b)`
解方程 `ax + b = 0`。若 `|a| < EPSILON` 则抛出异常。返回单根 `-b/a`。

#### 2.2 一元二次方程 `solve2(a, b, c)`
解方程 `ax² + bx + c = 0`。若 `|a| < EPSILON` 则降为一次处理。
- 计算判别式 `discriminant = b² - 4ac`。
- 若 `discriminant > 0`，返回两不等实根；若 `discriminant ≈ 0`，返回单重根；否则返回空数组（无实根）。

#### 2.3 一元三次方程 `solve3(a, b, c, d)`
解方程 `ax³ + bx² + cx + d = 0`。若 `|a| < EPSILON` 降为二次。
- 标准变换到 `x³ + px + q = 0`：
  ```
  p = (3ac - b²) / (3a²)
  q = (2b³ - 9abc + 27a²d) / (27a³)
  ```
- 判别式 `Δ = q²/4 + p³/27`。
- `Δ > 0`：一个实根，使用 Cardano 公式 `u = ∛(-q/2 + √Δ)`, `v = ∛(-q/2 - √Δ)`, 根为 `u+v - b/(3a)`。
- `Δ ≈ 0`：若 `q ≈ 0` 则为三重根 `-b/(3a)`；否则有一个单根和一个二重根，通过 `∛(q/2)` 计算。
- `Δ < 0`：三个实根，使用三角函数解法：
  ```
  r = √(-p/3)
  θ = acos( -q / (2r³) )  （需 clamp 到 [-1,1]）
  root1 = 2r cos(θ/3) - b/(3a)
  root2 = 2r cos((θ+2π)/3) - b/(3a)
  root3 = 2r cos((θ+4π)/3) - b/(3a)
  ```

#### 2.4 一元四次方程 `solve4(a, b, c, d, e)`
解方程 `ax⁴ + bx³ + cx² + dx + e = 0`。若 `|a| < EPSILON` 降为三次。
- 使用天珩求根公式的重判别式体系：
  ```
  D = 3b² - 8ac
  E = -b³ + 4abc - 8a²d
  F = 3b⁴ + 16a²c² - 16ab²c + 16a²bd - 64a³e
  A = D² - 3F
  B = DF - 9E²
  C = F² - 3DE²
  Δ = B² - 4AC
  ```
- 根据以上判别式依次判断六种情况：
  1. **D=E=F=0**：四重实根 `-b/(4a)`。
  2. **D*E*F≠0 且 A=B=C=0**：一个三重根和一个单根，具体公式：
     ```
     x1 = (-bD + 9E) / (4aD)
     x2 = (-bD - 3E) / (4aD)
     ```
  3. **E=F=0 且 D≠0**：两对二重根。若 D>0，两不等实根 `(-b±√D)/(4a)`；D<0 返回空（虚根）。
  4. **ABC≠0 且 Δ=0**：一对二重实根和两个单实根（仅当 AB>0，否则虚根），公式：
     ```
     x1,x2 = [-b + 2AE/B ± √(2B/A)] / (4a)
     x3 = [-b - 2AE/B] / (4a)   (二重根)
     ```
  5. **Δ>0**：两个不等实根和一对共轭虚根，只返回两实根。计算 `z1,z2` 及立方根，然后：
     ```
     realPart = [-b + sgn(E)*√((D+∛z1+∛z2)/3)] / (4a)
     imagPart = √[(2D - (∛z1+∛z2) + 2√z)/3] / (4a)
     x1 = realPart + imagPart
     x2 = realPart - imagPart
     ```
     其中 `z` 为中间量 `z = D² - D(∛z1+∛z2) + (∛z1+∛z2)² - 3A`。
  6. **Δ<0 且 D>0且F>0**：四个不等实根。进一步判断 E 是否为0：
     - 若 E=0，直接 `x = [-b ± √(D±2√F)]/(4a)`。
     - 若 E≠0，计算三角函数角度 `θ = acos((3B-2AD)/(2A√A))`，得到三个中间量 `y1,y2,y3`（其中 y2 为最大），然后：
       ```
       x1,x2 = [-b + sgn(E)√y1 ± (√y2 + √y3)]/(4a)
       x3,x4 = [-b - sgn(E)√y1 ± (√y2 - √y3)]/(4a)
       ```
- 所有其他情况均无实根，返回空数组。
- 注意处理立方根和开方时避免负值（对可能负数取 `Math.max(...,0)`），保证程序安全。

---

### 3. 最小二乘直线拟合

#### `public static Line fitLine(List<Point2D> points)`
- 要求点数至少2个，否则抛异常。
- 计算 `∑x, ∑y, ∑xy, ∑x², ∑y²`。
- 斜率：`slope = (n∑xy - ∑x∑y) / (n∑x² - (∑x)²)`（或基于均值的分子分母）。
- 截距：`intercept = meanY - slope * meanX`。
- 特殊情况：若所有 x 值相同（`denominator ≈ 0`），则直线垂直，返回斜率 `POSITIVE_INFINITY`，截距 `NaN`，R²=0。
- 计算决定系数 R²：
  ```
  ssTotal = Σ(y - meanY)²
  ssResidual = Σ(y - predicted)²
  R² = 1 - (ssResidual / ssTotal)
  ```
- 返回 `new Line(slope, intercept, rSquared)`。

---

### 4. 角度与坐标工具

#### `normalizeAngle(double angle)`
- 如果角度为 NaN 则返回 NaN。
- 将角度限制到 `(-π, π]`：采用循环加减 `2π`。
- 额外调用 `Point2D.fromPolar(angle,1).getRadian()` 以保证最后的结果精确在 `[-π, π]`（利用极坐标归一化）。

#### `toPose2d(Point2D point2D, double heading)`
- 将自定义的 `Point2D` 转换为 RoadRunner 的 `Pose2d`。
- 首先将点旋转 `-90°`（`Point2D.rotate(point2D, -90°)`）。
- 然后构造 `new Pose2d(rotation.getX(), rotation.getY(), heading)`。

#### `toPoint2D(Pose2d pose2d)`
- 反向转换，从 `Pose2d` 创建 `Point2D`，再旋转 `+90°`。

---

### 5. 数值工具

#### `sgn(double n)`
- 返回 `Math.signum(n)`。

#### `hypot(double a, double b)`
- 安全计算 `√(a² + b²)`，避免中间溢出。
- 使用公式：若 `|a| > |b|`，`r = b/a`，结果 `|a|*√(1+r²)`；否则类似处理。

#### `hypot(Number... numbers)`
- 多参数版本，迭代调用 `Math.hypot`。

#### `avg(double... doubles)` / `avg(Number... numbers)`
- 返回参数的平均值，为空时返回 0.0。

#### `toInch(double mm)` 和 `toMM(double inch)`
- 单位换算：`inch = mm * 0.0394`，`mm = inch * 25.4`。

---

### 6. 全局常量

- `EPSILON = 1e-10`：用于浮点数比较的容差。

---

### 7. 使用示例

```java
// 解方程 2x^3 - 4x^2 - 22x + 24 = 0
double[] roots = MathSolver.solve3(2, -4, -22, 24); // 应得 4, -3, 1

// 角度归一化
double angle = MathSolver.normalizeAngle(5.0 * Math.PI); // 约 -3.1415

// 直线拟合
List<Point2D> pts = ...;
Line line = MathSolver.fitLine(pts);
System.out.println(line.getEquation() + ", R²=" + line.getRSquared());

// 坐标转换
Pose2d roadRunnerPose = MathSolver.toPose2d(point, 0.0);
Point2D back = MathSolver.toPoint2D(roadRunnerPose);
```

---

### 8. 实现注意事项

- 多项式求根时，严格遵循降阶逻辑（四次→三次→二次→一次）。
- 四次公式中许多步骤需要检查数值合法性，避免 `NaN` 或负数的平方根/立方根。
- `sgn(E)` 用于处理 E 的符号因子（E=0 时为 0）。
- 最小二乘法要处理垂直线的退化情况。
- 所有方法应为静态方法，类不可实例化（构造函数私有或省略）。
- 依赖的外部类：`Line`, `Point2D`（自定义），`Pose2d`（RoadRunner）。
- 代码注释应包含公式出处（如天珩公式）以保证可维护性。

---

请根据以上规范，实现完整的 `MathSolver` 工具类，确保数值稳定性和全面的文档注释。