## Jama 矩阵库与一维卡尔曼滤波器实现规范 (LLM Prompt)

你是一位数值计算与机器人状态估计工程师。请实现一套基于 **Jama 矩阵库**的线性代数工具包，并在此基础上构建一个**一维位置-速度卡尔曼滤波器**，用于融合轮式里程计与外部绝对定位（如视觉）数据。该库包含矩阵基本运算、五种矩阵分解以及一个一维卡尔曼滤波器。

---

### 第一部分：Matrix 类（核心矩阵类）

#### 1.1 功能概述
实现一个名为 `Matrix` 的 Java 类，提供实数矩阵的基本运算、线性代数求解和多种矩阵分解接口。该类设计参考 JAMA（Java Matrix）库，支持：
- 多种构造函数从不同数据源创建矩阵。
- 元素访问、子矩阵提取与赋值。
- 矩阵算术：加法、减法、乘法（标量与矩阵）、逐元素乘除。
- 矩阵转置、范数（1-范数、2-范数、无穷范数、Frobenius 范数）。
- 线性方程求解 `A*X = B` 和 `X*A = B`。
- 矩阵逆、行列式、秩、条件数、迹。
- 五种分解：LU、QR、Cholesky、特征值、奇异值。
- 矩阵打印和流式读取功能。

#### 1.2 数据结构
- `private double[][] A` – 内部二维数组存储矩阵元素。
- `private int m, n` – 行数和列数。

#### 1.3 构造函数
- `Matrix(int m, int n)` – 创建 m×n 零矩阵。
- `Matrix(int m, int n, double s)` – 创建 m×n 常数矩阵，所有元素为 `s`。
- `Matrix(double[][] A)` – 从二维数组构造（检查各行长度一致）。
- `Matrix(double[][] A, int m, int n)` – 快速构造（不检查参数合法性，供内部类使用）。
- `Matrix(double vals[], int m)` – 从按列打包的一维数组构造（Fortran 风格）。

#### 1.4 静态工厂方法
- `static Matrix constructWithCopy(double[][] A)` – 通过深拷贝二维数组构造。
- `static Matrix random(int m, int n)` – 生成随机矩阵（元素在 [0,1) 均匀分布）。
- `static Matrix identity(int m, int n)` – 生成单位矩阵（主对角线为 1，其余为 0）。

#### 1.5 基本操作
- `copy()` / `clone()` – 深拷贝。
- `getArray()` – 返回内部数组引用。
- `getArrayCopy()` – 返回内部数组的深拷贝。
- `getColumnPackedCopy()` / `getRowPackedCopy()` – 按列/行打包为一维数组。
- `getRowDimension()` / `getColumnDimension()` – 返回 m、n。
- `get(int i, int j)` / `set(int i, int j, double s)` – 获取/设置单元素。
- 子矩阵操作：`getMatrix(...)`, `setMatrix(...)` 多种重载（支持矩形索引和数组索引）。

#### 1.6 矩阵运算
- `transpose()` – 返回转置 `A'`。
- `norm1()` – 1-范数（最大列和）。
- `norm2()` – 2-范数（最大奇异值）。
- `normInf()` – 无穷范数（最大行和）。
- `normF()` – Frobenius 范数（所有元素平方和的平方根，使用 `MathSolver.hypot` 安全累加）。
- `uminus()` – 一元负号 `-A`。
- `plus(Matrix B)` / `plusEquals(Matrix B)` – 矩阵加法。
- `minus(Matrix B)` / `minusEquals(Matrix B)` – 矩阵减法。
- `times(double s)` / `timesEquals(double s)` – 标量乘法。
- `times(Matrix B)` – 矩阵乘法。
- `arrayTimes(Matrix B)` / `arrayTimesEquals(Matrix B)` – 逐元素乘法 `A.*B`。
- `arrayRightDivide(Matrix B)` / `arrayRightDivideEquals(Matrix B)` – 逐元素右除 `A./B`。
- `arrayLeftDivide(Matrix B)` / `arrayLeftDivideEquals(Matrix B)` – 逐元素左除 `A.\B`。

#### 1.7 分解方法
- `lu()` – 返回 `LUDecomposition` 对象。
- `qr()` – 返回 `QRDecomposition` 对象。
- `chol()` – 返回 `CholeskyDecomposition` 对象。
- `svd()` – 返回 `SingularValueDecomposition` 对象。
- `eig()` – 返回 `EigenvalueDecomposition` 对象。

#### 1.8 高级求解器
- `solve(Matrix B)` – 求解 `A*X = B`：若 A 方阵则用 LU 分解，否则用 QR 分解（最小二乘解）。
- `solveTranspose(Matrix B)` – 求解 `X*A = B`，等价于 `A'*X' = B'`。
- `inverse()` – 矩阵逆（方阵）或伪逆。
- `det()` – 行列式。
- `rank()` – 有效数值秩（基于 SVD）。
- `cond()` – 条件数（2-范数）。
- `trace()` – 迹（主对角线元素之和）。

#### 1.9 输入/输出
- `print(int w, int d)` / `print(PrintWriter output, int w, int d)` – 打印矩阵，指定列宽和小数位数。
- `print(NumberFormat format, int width)` / `print(PrintWriter output, NumberFormat format, int width)` – 使用格式化对象打印。
- `static Matrix read(BufferedReader input)` – 从流中读取矩阵（格式与 `print` 输出匹配）。

#### 1.10 内部方法
- `private void checkMatrixDimensions(Matrix B)` – 检查两个矩阵的行列数是否相同，否则抛出 `IllegalArgumentException`。

#### 1.11 异常处理
- 构造函数 `Matrix(double[][] A)` 检查各行长度相同。
- 矩阵乘法检查内部维度匹配。
- `solve` 等分解方法可能抛出 "Matrix is singular" 等异常，由对应的分解类处理。

---

### 第二部分：矩阵分解类

#### 2.1 LUDecomposition（LU 分解）

对 m×n 矩阵 A，计算 LU 分解，使得 `A(piv,:) = L*U`，其中 L 为 m×min(m,n) 单位下三角矩阵，U 为 min(m,n)×n 上三角矩阵，`piv` 为 m 维置换向量。

**字段**：
- `private double[][] LU` – 内部存储 L 和 U 的合并矩阵。
- `private int m, n` – 行、列维度。
- `private int pivsign` – 置换符号（+1 或 -1）。
- `private int[] piv` – 置换向量。

**构造方法**：
- `LUDecomposition(Matrix A)` – 使用 Crout/Doolittle 算法计算 LU 分解（左视点积方法）：
  1. 复制 A 到 `LU`。
  2. 初始化 `piv` 为恒等映射。
  3. 对每一列 j，复制该列到 `LUcolj`，应用之前计算的变换，查找主元并交换，计算乘数并存储。
  4. 选取列主元（最大绝对值元素）以提高数值稳定性。

**公共方法**：
- `boolean isNonsingular()` – 检查 U 对角线上是否有零元素。
- `Matrix getL()` – 提取下三角因子 L。
- `Matrix getU()` – 提取上三角因子 U。
- `int[] getPivot()` – 返回置换向量的副本。
- `double[] getDoublePivot()` – 以 double 数组形式返回置换向量。
- `double det()` – 计算行列式（必须方阵，否则抛异常）。
- `Matrix solve(Matrix B)` – 求解 `A*X = B`：先置换 B 的行，前代求解 `L*Y = B(piv,:)`，回代求解 `U*X = Y`。

---

#### 2.2 QRDecomposition（QR 分解）

对 m×n 矩阵 A (m≥n)，计算 QR 分解，使得 `A = Q*R`，其中 Q 正交，R 上三角。

**字段**：
- `private double[][] QR` – 内部存储 Householder 向量和 R 的上三角部分。
- `private int m, n` – 行、列维度。
- `private double[] Rdiag` – 存储 R 的对角线元素。

**构造方法**：
- `QRDecomposition(Matrix A)` – 使用 Householder 反射计算 QR 分解：
  1. 复制 A 到 `QR`。
  2. 对于 k = 0 到 n-1，计算第 k 列的 2-范数（安全版本），构造 Householder 向量使该列除对角线外全零，并更新剩余的列。

**公共方法**：
- `boolean isFullRank()` – 检查 R 对角线是否有零。
- `Matrix getH()` – 返回 Householder 向量矩阵（下梯形）。
- `Matrix getR()` – 返回上三角因子 R。
- `Matrix getQ()` – 生成并返回（经济大小的）正交因子 Q。
- `Matrix solve(Matrix B)` – 最小二乘解：计算 `Y = Q'*B`，然后通过回代求解 `R*X = Y`。

---

#### 2.3 CholeskyDecomposition（Cholesky 分解）

对对称正定矩阵 A，计算 Cholesky 分解 `A = L*L'`，其中 L 为下三角矩阵。

**字段**：
- `private double[][] L` – 内部存储下三角因子。
- `private int n` – 维度（方阵）。
- `private boolean isspd` – 是否对称正定。

**构造方法**：
- `CholeskyDecomposition(Matrix Arg)` – 执行 Cholesky 算法：
  1. 初始化 L 为零矩阵，假设 A 为方阵。
  2. 对每一列 j，计算 L[j][k] = (A[j][k] - ΣL[k][i]*L[j][i]) / L[k][k]（k<j）。
  3. 计算对角元 L[j][j] = sqrt(A[j][j] - Σ(L[j][k])²)。
  4. 同时更新 `isspd`：检查 A 是否对称及对角线元素是否正定。

**公共方法**：
- `boolean isSPD()` – 返回对称正定标志。
- `Matrix getL()` – 返回下三角因子 L。
- `Matrix solve(Matrix B)` – 求解 `A*X = B`：先前代求解 `L*Y = B`，再回代求解 `L'*X = Y`。

---

#### 2.4 EigenvalueDecomposition（特征值分解）

对实方阵 A，若对称则 `A = V*D*V'`，D 对角，V 正交；若不对称，D 为块对角（1×1 实特征值或 2×2 共轭复数对）。

**字段**：
- `private int n` – 矩阵维度。
- `private boolean issymmetric` – 是否对称。
- `private double[] d, e` – 特征值的实部和虚部。
- `private double[][] V` – 特征向量矩阵。
- `private double[][] H` – 非对称情况下的上 Hessenberg 形式。
- `private double[] ort` – 非对称算法的工作存储。

**构造方法**：
- `EigenvalueDecomposition(Matrix Arg)` – 判断对称性：
  - 若对称 → 复制到 V，调用 `tred2()` 三对角化，再调用 `tql2()` QL 算法对角化。
  - 若不对称 → 复制到 H，调用 `orthes()` Hessenberg 化，再调用 `hqr2()` 化为实 Schur 形式。

**私有方法**：
- `tred2()` – Householder 三对角化（对称情形）。
- `tql2()` – QL 隐式对称三对角算法。
- `orthes()` – 非对称 Hessenberg 化。
- `hqr2()` – 非对称 QR 算法（多步迭代，含 Wilkinson 移位和 MATLAB ad hoc 移位）。
- `cdiv(double xr, double xi, double yr, double yi)` – 复数除法辅助方法。

**公共方法**：
- `Matrix getV()` – 返回特征向量矩阵。
- `double[] getRealEigenvalues()` – 返回实部数组。
- `double[] getImagEigenvalues()` – 返回虚部数组。
- `Matrix getD()` – 返回块对角特征值矩阵。

---

#### 2.5 SingularValueDecomposition（奇异值分解）

对 m×n 矩阵 A，计算 SVD `A = U*S*V'`，其中 U 和 V 正交，S 对角且元素非负递减。

**字段**：
- `private double[][] U, V` – 左右奇异向量矩阵。
- `private double[] s` – 奇异值数组。
- `private int m, n` – 维度。

**构造方法**：
- `SingularValueDecomposition(Matrix Arg)` – 基于 LINPACK 代码：
  1. 预处理A为双对角形式，存储对角元素到 s、超对角到 e。
  2. 生成 U 和 V。
  3. 主迭代循环：扫描可忽略元素与分裂，根据四种情况执行 qr 步骤，收敛后排序使奇异值非递增。
  4. 截断至计算精度。

**公共方法**：
- `Matrix getU()` – 返回左奇异向量矩阵。
- `Matrix getV()` – 返回右奇异向量矩阵。
- `double[] getSingularValues()` – 返回奇异值数组。
- `Matrix getS()` – 返回对角矩阵 S。
- `double norm2()` – 返回最大奇异值。
- `double cond()` – 返回条件数（最大/最小奇异值）。
- `int rank()` – 返回有效数值秩。

---

### 第三部分：矩阵依赖关系

以上所有分解类均依赖 `Matrix` 类，并在构造时调用 `Matrix.getArray()`、`getArrayCopy()` 等方法获取底层数据。它们通过包内可见性（同一包内）直接向 `Matrix` 的构造函数传递原始数组，以实现高效返回结果（例如 `new Matrix(L, n, n)`）。

一些分解类（如 QR、SVD、Matrix.normF）也依赖 `MathSolver.hypot()` 方法安全计算 `sqrt(a²+b²)`，请在项目中提供该工具（见前文 MathSolver 规范）。

---

### 第四部分：OneDimensionKalmanFilter（一维卡尔曼滤波器）

#### 4.1 功能概述
实现一个一维位置-速度卡尔曼滤波器，状态向量为 `[position, velocity]`，使用常速模型融合轮式里程计增量与外部绝对定位（如视觉位姿）测量。支持在线调整过程噪声和测量噪声参数。

#### 4.2 数据结构
- `long lastUpdateTime` – 上次更新时间（纳秒），用于计算 `dt`。
- `double lastEsPosition, lastEsVelocity, lastWheelPosition` – 上一次估计的位置、速度及轮式里程计位置。
- `Matrix F` – 状态转移矩阵，在 `updateF(double dt)` 中动态设置：`[[1, dt], [0, 0.95]]`（速度轻微衰减，增加鲁棒性）。
- `Matrix P` – 状态协方差矩阵，初始为 `[[0.01, 0], [0, 0.001]]`。
- `Matrix H` – 观测矩阵，固定为 `[[1, 0]]`（仅测量位置）。
- `static double q_pos, q_vel` – 过程噪声（Dashboard 可调），默认 `q_pos=1e-4, q_vel=1e-6`。
- `static double R` – 测量噪声方差，默认 `0.01²`.
- `Matrix Q` – 过程噪声协方差矩阵 `[[q_pos, 0], [0, q_vel]]`。

#### 4.3 构造方法
- `OneDimensionKalmanFilter(double initialPosition, double initialVelocity)`
  - 初始化状态和轮式里程计为 `initialPosition`。
  - 速度初始化为 `initialVelocity`。
  - 初始化 F 为单位矩阵（首次更新前不会使用）。

#### 4.4 核心方法：`PosVelTuple Update(double wheelPosition, double measurementPosition)`
1. **计算时间增量**：`dt = (currentNanoTime - lastUpdateTime) / 1e9` 秒，更新 `lastUpdateTime`。
2. **计算轮式里程计增量**：`deltaPosition = wheelPosition - lastWheelPosition`，更新 `lastWheelPosition`。
3. **构建 F 矩阵**：调用 `updateF(dt)`。
4. **先验状态预测 X_**：
   ```
   X_ = F * [lastEsPosition + deltaPosition]
            [deltaPosition / dt       ]
   ```
   含义：位置直接用轮式增量修正，速度由里程计增量除以时间获得。
5. **更新 Q 矩阵**：从静态变量 `q_pos`、`q_vel` 赋值到 Q 对角线。
6. **先验协方差预测**：`P_ = F * P * F' + Q`。
7. **若无视觉测量** (`measurementPosition` 为 `NaN`)：
   - 仅保存预测状态，不进行更新。
   - 设置 `P = P_`。
   - 返回预测的 `(position, velocity)`。
8. **有视觉测量**：
   - 计算卡尔曼增益：
     ```
     K = P_ * H' / (H * P_ * H' + R)
     ```
     注意 `H*P_*H'` 返回 1×1 矩阵，取 `get(0,0)` 并使用标量倒数。
   - 更新状态：`X = X_ + K * (measurementPosition - H*X_)`。
   - 更新协方差：`P = (I - K*H) * P_`。
   - 存储 `lastEsPosition = X(0,0)`, `lastEsVelocity = X(1,0)`。
   - 返回 `(position, velocity)`。

#### 4.5 重置方法
- `void reset(double position, double velocity)`
  - 重置状态、里程计、时间戳、协方差矩阵 P 为初始值，F 为单位矩阵。

#### 4.6 `PosVelTuple` 辅助类
简单数据容器，包含 `public final double position;` 和 `public final double velocity;`，通过构造函数赋值。

---

### 第五部分：整体打包说明

所有类应组织在对应的包中：
- `org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.jama` 包含：
  - `Matrix.java`
  - `LUDecomposition.java`
  - `QRDecomposition.java`
  - `CholeskyDecomposition.java`
  - `EigenvalueDecomposition.java`
  - `SingularValueDecomposition.java`
- `org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter` 包含：
  - `OneDimensionKalmanFilter.java`
  - `PosVelTuple.java`

依赖项：
- 所有类依赖 `MathSolver` 工具类中的 `hypot(double, double)` 方法（见前文 MathSolver 规范）。其中 `Matrix.normF()` 使用 `MathSolver.hypot(f, A[i][j])` 来安全累积平方和；`QRDecomposition`、`SingularValueDecomposition` 中也使用了类似的 hypot 计算。
- 卡尔曼滤波器依赖 `Matrix` 类进行运算。
- 可使用 `@Config` 注解支持 Dashboard 在线调参（`q_pos`, `q_vel`, `R`）。

---

### 第六部分：使用示例

```java
// 创建矩阵并进行运算
double[][] vals = {{1,2,3},{4,5,6},{7,8,10}};
Matrix A = new Matrix(vals);
Matrix b = Matrix.random(3,1);
Matrix x = A.solve(b);
double det = A.det();

// 卡尔曼滤波
OneDimensionKalmanFilter kf = new OneDimensionKalmanFilter(0.0, 0.0);

// 在循环中：轮式里程计读数 wheelPos，视觉读数 visionPos（无则为 NaN）
PosVelTuple result = kf.Update(wheelPos, visionPos);
double estimatedPos = result.position;
double estimatedVel = result.velocity;
```

---

请根据以上规范，同时生成所有上述 Java 类的完整代码，确保数值稳定性、算法正确性，并包含详细的 Javadoc 注释。