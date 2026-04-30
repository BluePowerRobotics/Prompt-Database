## 滑动窗口/移动平均滤波器组实现规范 (LLM Prompt)

你是一位信号处理或机器人软件工程师。请实现一套**滑动窗口滤波器**，用于对传感器数据（如编码器速度、IMU角度）进行实时平滑降噪。该滤波器组包括三个类：

1. **`MeanFilter`** – 标量数据的滑动算术平均滤波器。
2. **`AngleMeanFilter`** – 角度数据的滑动平均滤波器，采用单位向量加权法避免角度环绕（如 -π 到 π 的跳变）。
3. **`AngleWeightedMeanFilter`** – 带权重的角度滑动平均滤波器，角度本身可以附加权值（如信号置信度），以向量加权求和方式计算平均角。

所有滤波器实现**循环缓冲区**，支持增量更新，时间复杂度 O(1)。依赖自定义的 `Point2D` 类实现向量运算（需单独实现，包含 `fromPolar`、`translate`、`getRadian`、`equal` 等方法）。

---

### 1. 通用设计原则

- 构造时指定窗口大小 `windowSize > 0`，否则抛出 `IllegalArgumentException`。
- 维护一个循环缓冲区，长度为 `windowSize`，以及当前有效样本数 `count`（≤ `windowSize`）。
- `index` 指向下一个写入位置。
- 新数据到达时，若缓冲区未满则直接追加；若已满则替换最旧元素。
- 提供 `reset()` 清空历史数据，提供获取当前平均值的 getter。
- 处理非法输入（NaN、无穷大等）时不更新状态，返回当前的均值。

---

### 2. `MeanFilter` – 普通标量均值滤波器

#### 2.1 字段
- `windowSize` (final int)
- `buffer` (double[]) 长度为 `windowSize`
- `index` (int) 写入位置
- `count` (int) 已接收样本数
- `sum` (double) 当前窗口内所有值的和

#### 2.2 核心方法

##### `public double filter(double newValue)`
1. **输入验证**：如果 `newValue` 为 NaN 或无穷大，直接返回 `getMean()`（不更新缓冲区）。
2. **未满状态** (`count < windowSize`)：
   - 将 `newValue` 存入 `buffer[index]`。
   - `sum += newValue`。
   - `count++`。
   - `index = (index + 1) % windowSize`。
   - 返回 `sum / count`。
3. **已满状态**：
   - 减去最旧元素：`sum -= buffer[index]`。
   - 存入新值：`buffer[index] = newValue`。
   - `sum += newValue`。
   - `index = (index + 1) % windowSize`。
   - 返回 `sum / windowSize`。

##### `public void reset()`
- 将 `buffer` 所有元素置 0，`index = 0`, `count = 0`, `sum = 0.0`。

##### `public double getMean()`
- 若 `count == 0` 则返回 0.0，否则返回 `sum / count`。

##### `public double getVariance()`
- 若 `count == 0` 返回 0.0。
- 计算当前窗口内所有样本的总体方差（除以 `count`）：`Σ(buffer[i] - mean)² / count`。

##### 其他
- `getCount()` 返回 `count`。
- `getWindowSize()` 返回 `windowSize`。

---

### 3. `AngleMeanFilter` – 角度均值滤波器（单位向量法）

#### 3.1 背景
直接对角度值求算术平均会在跨越 ±π 边界时产生错误结果（例如 -179° 和 179° 的平均应该是 180° 或 -180°，而不是 0°）。经典解法是将每个角度视为单位向量 `(cosθ, sinθ)`，求这些向量的和，再通过 `atan2` 计算平均方向。

#### 3.2 字段
- `windowSize` (final int)
- `unitVectors` (Point2D[]) 长度为 `windowSize`，每个元素初始化为 `(0,0)`
- `index` (int)
- `count` (int)
- `vectorSum` (Point2D) 当前所有向量的累加和，初始化为 `(0,0)`

#### 3.3 核心方法

##### `public double filter(double angleRad)`
1. 将输入角度转换为单位向量：`newVector = Point2D.fromPolar(angleRad, 1.0)`。
2. 如果缓冲区未满：
   - 存储 `unitVectors[index] = newVector`。
   - 如果 `newVector` 的弧度不是 NaN（即非零向量），累加：`vectorSum = Point2D.translate(vectorSum, newVector.getX(), newVector.getY())`。
   - `count++`, `index = (index + 1) % windowSize`。
3. 如果缓冲区已满：
   - 取出最旧向量 `oldest = unitVectors[index]`。
   - 若 `oldest` 弧度非 NaN，从和中减去：`vectorSum = Point2D.translate(vectorSum, -oldest.getX(), -oldest.getY())`。
   - 替换为新向量，同未满情况的累加逻辑。
   - 更新 `index`。
4. 如果最终 `vectorSum` 与 `Point2D.ZERO` 相等（即向量和为零），返回 `Double.NaN`。
5. 否则返回 `vectorSum.getRadian()`，即平均方向角（范围 `(-π, π]`）。

##### `public double filterDegrees(double angleDeg)`
- 将角度转为弧度调用 `filter()`，再将结果转为度数返回。

##### `public double getAverageAngle()`
- 若 `vectorSum` 为零返回 NaN，否则返回 `vectorSum.getRadian()`。

##### `public void reset()`
- 将所有 `unitVectors` 置为 `(0,0)`，`vectorSum` 置为 `(0,0)`，`index = 0`, `count = 0`。

##### `public double getConsistency()`
- 返回向量和的长度除以样本数（或窗口大小），取值范围 [0,1]；1 表示所有角度完全相同，0 表示均匀分散。公式：`vectorSum.getDistance() / Math.min(count, windowSize)`。

---

### 4. `AngleWeightedMeanFilter` – 带权重的角度均值滤波器

#### 4.1 与 `AngleMeanFilter` 的区别
每个角度样本可以附带一个**正权重**（如传感器置信度、距离的倒数等），而不再假设所有样本权重为 1。平均方向的计算方式为：将角度视为向量 `weight * (cosθ, sinθ)`，求所有向量的加权和，再通过 `atan2` 计算平均方向。

#### 4.2 字段
与 `AngleMeanFilter` 完全一致。

#### 4.3 核心方法

##### `public double filter(double angleRad, double weight)`
1. 构建向量：`newVector = Point2D.fromPolar(angleRad, weight)`，即长度为 `weight`，方向为 `angleRad`。
2. 缓冲区未满/已满的处理逻辑与普通角度滤波器相同，只是所有向量的模可能不是 1。累加和减法时仍使用相同的 `translate` 方法（加/减 X/Y 分量）。
3. 最终同样判断 `vectorSum` 是否为零，若是返回 NaN，否则返回 `vectorSum.getRadian()`。

##### `public double filterDegrees(double angleDeg, double weight)`
- 转换后调用 `filter(angleRad, weight)`，结果转为度数。

其他方法 `reset()`, `getAverageAngle()`, `getConsistency()` 与 `AngleMeanFilter` 相同。

---

### 5. 依赖要求

- **`Point2D` 类**必须提供以下静态/实例方法：
  - `Point2D(double x, double y)` – 构造函数。
  - `static Point2D ZERO` – 零点常量。
  - `static Point2D fromPolar(double radian, double distance)` – 从极坐标创建点。
  - `static Point2D translate(Point2D p, double dx, double dy)` – 平移点。
  - `double getRadian()` – 返回点的极角 `atan2(y, x)`。
  - `double getX()`, `double getY()`, `void setX(double)`, `void setY(double)` – 读写坐标。
  - `static boolean equal(Point2D p1, Point2D p2)` – 比较两点是否在容差内相等（容差可设为 `1e-10`）。

---

### 6. 使用示例

```java
// 标量均值滤波
MeanFilter speedFilter = new MeanFilter(5);
double smoothedSpeed = speedFilter.filter(encoder.getVelocity());

// 角度滤波 (弧度)
AngleMeanFilter headingFilter = new AngleMeanFilter(10);
double smoothedHeading = headingFilter.filter(imu.getYaw(AngleUnit.RADIANS));

// 加权角度滤波
AngleWeightedMeanFilter weightedFilter = new AngleWeightedMeanFilter(8);
double fusedHeading = weightedFilter.filter(visionHeading, visionConfidence);
```

---

请根据以上规范，生成三个类的完整 Java 代码，确保正确使用循环缓冲区，并处理好 NaN 和向量零点等特殊情况。所有代码应包含清晰的 Javadoc 注释。