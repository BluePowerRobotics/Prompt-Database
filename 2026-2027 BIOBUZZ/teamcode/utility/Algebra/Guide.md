# Algebra 代数计算工具

## 概述

本目录包含代数计算相关的工具类，主要用于方程求解、复数运算和单位转换。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `EquationSolver.java` | 多项式方程求解器（实数/复数），及单位转换、平均值计算 |
| `ComplexNumber.java` | 复数类，提供四则运算、开方/开立方、幅值等操作 |

## EquationSolver 使用说明

### 功能特性

- 一元一次方程求解 (`solve1`)
- 一元二次方程求解 (`solve2`)
- 一元三次方程求解 (`solve3`)
- 一元四次方程求解 (`solve4`)
- 复数四次方程求解 (`solve4Complex`)
- 从复数根中过滤实数根 (`filterRealRoots`)
- 单位转换方法（毫米↔英寸）
- 平均值计算方法

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Algebra.EquationSolver;
import org.firstinspires.ftc.teamcode.utility.Algebra.ComplexNumber;

// 求解二次方程: x^2 - 5x + 6 = 0
double[] roots = EquationSolver.solve2(1, -5, 6);
// 返回: [3.0, 2.0]

        // 求解三次方程: x^3 - 6x^2 + 11x - 6 = 0
        double[] cubicRoots = EquationSolver.solve3(1, -6, 11, -6);
// 返回: [1.0, 2.0, 3.0]

        // 复数四次方程求解（返回所有复数根，含重根）
        ComplexNumber[] complexRoots = EquationSolver.solve4Complex(a, b, c, d, e);

        // 从复数根中过滤出实数根
        double[] realRoots = EquationSolver.filterRealRoots(complexRoots);

        // 单位转换
        double inches = EquationSolver.toInch(100);  // 100mm -> 英寸
        double mm = EquationSolver.toMM(4);          // 4英寸 -> mm

        // 计算平均值
        double avg = EquationSolver.avg(1.0, 2.0, 3.0, 4.0);
```

### 方法签名

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `solve1(a, b)` | 求解 ax + b = 0 | `double[]` 根数组 |
| `solve2(a, b, c)` | 求解 ax² + bx + c = 0 | `double[]` 根数组 |
| `solve3(a, b, c, d)` | 求解 ax³ + bx² + cx + d = 0 | `double[]` 根数组 |
| `solve4(a, b, c, d, e)` | 求解 ax⁴ + bx³ + cx² + dx + e = 0 | `double[]` 根数组 |
| `solve4Complex(a, b, c, d, e)` | 求解复数四次方程 | `ComplexNumber[]` 复数根 |
| `filterRealRoots(complexRoots)` | 过滤虚部小于阈值的实数根 | `double[]` |
| `sgn(n)` | 返回 n 的符号 | `double` |
| `toInch(mm)` | 毫米转英寸 | `double` |
| `toMM(inch)` | 英寸转毫米 | `double` |
| `avg(doubles...)` / `avg(numbers...)` | 计算平均值 | `double` |

### 注意事项

1. 实数根数组可能为空（无实数解）
2. 高阶方程求解使用数值方法，可能存在精度误差
3. 系数为0时会自动降级到低阶方程求解
4. `solve4Complex` 返回全部 4 个复数根（可能含重根），可通过 `filterRealRoots` 提取实数根

## ComplexNumber 使用说明

### 功能特性

- 复数四则运算：加 `add`、减 `subtract`、乘 `multiply`、除 `divide`
- 标量乘法 `multiply(scalar)`
- 求幅值 `magnitude`、平方根 `sqrt`、立方根 `cbrt`
- 取负 `negate`、判断是否为实数 `isReal`、提取实部 `toReal`

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.Algebra.ComplexNumber;

ComplexNumber z1 = new ComplexNumber(1, 2);
ComplexNumber z2 = new ComplexNumber(3, -4);

ComplexNumber sum = z1.add(z2);        // (4, -2)
ComplexNumber product = z1.multiply(z2); // (11, 2)
double magnitude = z1.magnitude();     // sqrt(5)
ComplexNumber sqrtZ = z1.sqrt();
boolean isReal = z1.isReal();          // false
```

### 方法签名

| 方法 | 说明 | 返回值 |
|------|------|--------|
| `ComplexNumber(real, imag)` | 构造函数 | `ComplexNumber` |
| `add(other)` / `subtract(other)` | 复数加减法 | `ComplexNumber` |
| `multiply(other)` / `divide(other)` | 复数乘除法 | `ComplexNumber` |
| `multiply(scalar)` | 标量乘法 | `ComplexNumber` |
| `magnitude()` | 求模长 | `double` |
| `sqrt()` / `cbrt()` | 开平方/开立方 | `ComplexNumber` |
| `negate()` | 取相反数 | `ComplexNumber` |
| `isReal()` | 虚部是否接近0 | `boolean` |
| `toReal()` | 返回实部 | `double` |

### 注意事项

1. 复数除法未对除数为零做保护，调用前需保证分母幅值非零
2. `isReal()` 以 `1e-10` 作为虚部阈值判断是否为实数