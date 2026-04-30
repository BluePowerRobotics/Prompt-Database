## 直线模型（Line）实现规范 (LLM Prompt)

你是一位软件开发工程师，需要实现一个**二维直线模型类 `Line`**，用于表示一元线性方程 `y = slope * x + intercept`，并提供拟合优度、预测、距离计算以及交点求解等功能。该类可用于数据分析、传感器标定、回归拟合等场景。

---

### 1. 功能概述

- 存储直线斜率 `slope`、截距 `intercept` 和拟合优度 `rSquared`。
- 根据给定的 `x` 值预测对应的 `y` 值。
- 返回直线方程的格式化字符串。
- 计算直线与坐标轴的交点。
- 计算空间一点到直线的垂直距离。
- 提供所有字段的 Getter 方法。

---

### 2. 构造方法

```java
public Line(double slope, double intercept, double rSquared)
```

- `slope`：直线的斜率（若为 `NaN` 表示无法拟合）。
- `intercept`：y 轴截距。
- `rSquared`：决定系数 R²，表示拟合优度（0~1，越大越好）。

---

### 3. 核心方法

#### 3.1 `public double predict(double x)`
- 根据方程 `y = slope * x + intercept` 返回预测值。

#### 3.2 `public String getEquation()`
- 如果 `slope` 或 `intercept` 为 `NaN`，返回 `"无法拟合直线"`。
- 否则返回形如 `"y = 2.5000x - 1.2000"` 的格式化字符串，注意：
  - 截距部分根据正负显示 `" + "` 或 `" - "`，数值部分取绝对值。
  - 系数保留四位小数。

#### 3.3 `public Double getXIntercept()`
- 返回直线与 X 轴的交点 x 坐标（即 y=0 时的 x 值）。
- 若斜率绝对值小于 `1e-10`（视为水平线），无交点，返回 `null`。
- 计算公式：`-intercept / slope`。

#### 3.4 `public double getYIntercept()`
- 直接返回 `intercept`，即直线与 Y 轴的交点。

#### 3.5 `public double distanceToPoint(double x, double y)`
- 计算点 `(x, y)` 到直线 `y = slope * x + intercept`（即 `slope * x - y + intercept = 0`）的**垂直距离**。
- 距离公式：`|slope * x - y + intercept| / √(slope² + 1)`。

---

### 4. Getter 与 toString

- `getSlope()`, `getIntercept()`, `getRSquared()` 分别返回对应字段。
- `toString()` 格式示例：`"Line{y = 2.5000x - 1.2000, R²=0.9987}"`，其中 R² 保留四位小数。

---

### 5. 使用示例

```java
// 创建一条斜率为2.5，截距为-1.2，R²为0.998的直线
Line line = new Line(2.5, -1.2, 0.9987);

// 预测 x=3 时的 y 值
double yPred = line.predict(3.0);  // 6.3

// 获取方程
System.out.println(line.getEquation()); // y = 2.5000x - 1.2000

// 计算点(4, 10)到直线的距离
double dist = line.distanceToPoint(4, 10); // |2.5*4 -10 -1.2| / sqrt(7.25) = ...

// 输出描述
System.out.println(line); // Line{y = 2.5000x - 1.2000, R²=0.9987}
```

---

### 6. 实现注意事项

- 所有数值使用 `double` 类型，注意处理 `NaN` 和无穷大。
- 斜率接近 0 时判断 `getXIntercept()` 返回 `null`，避免除以接近 0 的数。
- 距离计算的分母 `sqrt(slope*slope + 1)` 永远大于等于 1，无需额外处理。
- 格式化字符串使用 `String.format("%.4f", value)` 保证小数位数。
- 类的设计保持不可变性（字段均为 `final`），线程安全。

---

请根据以上规范，生成 Java 类 `Line`。代码应干净、高效，并包含完整的 Javadoc 注释。