# filter 数据滤波算法

## 概述

本目录包含数据滤波相关的工具类，用于处理传感器数据的噪声过滤。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `MeanFilter.java` | 移动平均滤波器 |
| `AngleMeanFilter.java` | 角度专用移动平均滤波器（避免角度跳变） |
| `AngleWeightedMeanFilter.java` | 加权角度移动平均滤波器 |
| `EMA.java` | 指数移动平均滤波器 |

## 滤波器选择指南

| 场景 | 推荐滤波器 | 原因 |
|------|-----------|------|
| 普通传感器数据滤波 | `MeanFilter` | 简单高效，适用于大多数场景 |
| 角度数据滤波 | `AngleMeanFilter` | 处理角度周期性，避免±180°跳变 |
| 带权重的角度滤波 | `AngleWeightedMeanFilter` | 可根据置信度加权 |
| 需要低延迟、平滑输出 | `EMA` | 单值递归，内存占用小、响应可调 |

## MeanFilter 使用说明

### 功能特性

- 滑动窗口移动平均
- 自动处理NaN和无穷大值（无效值丢弃并返回当前均值）
- 提供方差、样本数等统计信息

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

// 创建窗口大小为10的滤波器
MeanFilter filter = new MeanFilter(10);

// 逐步添加数据并获取滤波后的值
double filteredValue = filter.filter(rawSensorValue);

// 获取当前均值
double mean = filter.getMean();

// 获取方差
double variance = filter.getVariance();

// 已接收样本数与窗口大小
int count = filter.getCount();
int window = filter.getWindowSize();

// 重置滤波器
filter.reset();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `MeanFilter(windowSize)` | 构造函数，windowSize 必须大于0 |
| `filter(newValue)` | 添加新样本并返回滤波后值 |
| `getMean()` | 获取当前均值 |
| `getVariance()` | 获取当前方差 |
| `getCount()` | 获取已接收样本数 |
| `getWindowSize()` | 获取窗口大小 |
| `reset()` | 重置滤波器 |

## AngleMeanFilter 使用说明

### 功能特性

- 使用向量平均避免角度跳变问题
- 支持弧度和角度两种输入方式
- 提供一致性指标

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;

// 创建窗口大小为5的角度滤波器
AngleMeanFilter angleFilter = new AngleMeanFilter(5);

// 输入弧度
double filteredRad = angleFilter.filter(angleRad);

// 输入角度（度数）
double filteredDeg = angleFilter.filterDegrees(angleDeg);

// 获取当前平均角度
double avgAngle = angleFilter.getAverageAngle();

// 获取一致性指标（0-1，越接近1越一致）
double consistency = angleFilter.getConsistency();

// 重置滤波器
angleFilter.reset();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `AngleMeanFilter(windowSize)` | 构造函数，windowSize 必须大于0 |
| `filter(angleRad)` | 添加弧度值并返回平均角度 |
| `filterDegrees(angleDeg)` | 添加角度值并返回平均角度（度数） |
| `getAverageAngle()` | 获取当前平均角度 |
| `getConsistency()` | 获取一致性指标 |
| `reset()` | 重置滤波器 |

## AngleWeightedMeanFilter 使用说明

### 功能特性

- 支持加权角度平均
- 可根据传感器置信度动态调整权重

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleWeightedMeanFilter;

AngleWeightedMeanFilter filter = new AngleWeightedMeanFilter(5);

// 带权重的滤波（weight 作为向量的模长参与平均）
double filtered = filter.filter(angleRad, confidence);

// 带权重的度数滤波
double filteredDeg = filter.filterDegrees(angleDeg, confidence);

// 获取当前平均角度与一致性指标
double avgAngle = filter.getAverageAngle();
double consistency = filter.getConsistency();

// 重置滤波器
filter.reset();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `AngleWeightedMeanFilter(windowSize)` | 构造函数，windowSize 必须大于0 |
| `filter(angleRad, weight)` | 添加带权重的弧度值并返回平均角度 |
| `filterDegrees(angleDeg, weight)` | 添加带权重的角度值并返回平均角度（度数） |
| `getAverageAngle()` | 获取当前平均角度 |
| `getConsistency()` | 获取一致性指标 |
| `reset()` | 重置滤波器 |

## EMA 使用说明

### 功能特性

- 指数移动平均，公式 `filteredValue = alpha * x + (1 - alpha) * filteredValue`
- `alpha` 越接近1，平滑越弱、响应越快；越接近0，平滑越强、响应越慢
- 支持按等效窗口大小创建（`alpha = 2 / (n + 1)`）

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.EMA;

// 直接指定平滑系数
EMA ema = new EMA(0.2);

// 或按等效窗口大小创建
EMA ema2 = EMA.fromWindow(10);

// 逐步更新并获取滤波值（首次调用返回输入值作为初值）
double filtered = ema.update(rawSensorValue);

// 运行时调整平滑系数
ema.setAlpha(0.1);

// 获取当前状态
double value = ema.getFilteredValue();
double alpha = ema.getAlpha();
boolean hasInit = ema.hasInitialValue();

// 重置滤波器
ema.reset();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `EMA(alpha)` | 构造函数，alpha 范围 (0, 1] |
| `fromWindow(n)` | 按等效窗口大小 n 创建实例 |
| `update(x)` | 输入原始值，返回滤波后值 |
| `setAlpha(alpha)` | 设置平滑系数 |
| `getFilteredValue()` | 获取当前滤波值 |
| `getAlpha()` | 获取平滑系数 |
| `hasInitialValue()` | 是否已接收过有效样本 |
| `reset()` | 重置滤波器 |

### 注意事项

1. 所有滤波器对 NaN/无穷大值有不同处理：`MeanFilter` 与 `EMA` 会忽略无效值，`AngleMeanFilter`/`AngleWeightedMeanFilter` 会在向量和为零时返回 `NaN`
2. `windowSize` 必须大于 0，否则构造函数抛出 `IllegalArgumentException`
3. `EMA` 的 `alpha` 必须在 (0, 1] 范围内，否则抛出 `IllegalArgumentException`