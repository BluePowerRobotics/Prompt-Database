## 一、检测获取与坐标投影（`getCurrentDetections()`）

1. 通过 `detector` 分别获取紫色与绿色物体的**画面偏移量**（`purpleOffsets`、`greenOffsets`）。
2. 对每个偏移量，调用 `projector.project(offset[0], offset[1])` 将其转换为相对于摄像头的**世界坐标系坐标** `(x, y)`。
3. 封装为 `Detection` 对象（含颜色、坐标），存入列表返回。

> 输出：本帧所有可识别物体的世界坐标列表。

***

## 二、空间聚类分组（`clusterDetections()`）

**目的**：将空间上邻近的检测点聚合成一个候选组，作为潜在目标。

**算法**：基于距离阈值的**无向图连通分量搜索（BFS）**

1. 初始化 `processed` 集合记录已归类的检测。
2. 遍历所有 `Detection`：
   - 若已处理则跳过。
   - 新建空组，将当前检测入队。
   - BFS 循环：
     - 弹出队列头部元素，检查所有未处理检测。
     - 若两者距离 ≤ `distanceThreshold`，则加入同一组并入队。
3. 每一连通分量作为一个 `List<Detection>` 存入 `groups` 返回。

> 注：孤立的检测会形成大小为 1 的组。

***

## 三、将当前分组与现有目标匹配（`matchAndUpdateTargets()`）

### 1. 计算分组中心坐标

对每个组内所有检测的 `(x, y)` 取算术平均，得到 `(centerX, centerY)`。

### 2. 匹配现有 `Target`

- 遍历 `targets` 列表，寻找与组中心距离 ≤ `distanceThreshold` 且**距离最近**的目标。
- 若找到：
  - 调用 `matchedTarget.updateGroupAndMembers(group, confirmationFrames)` 更新成员。
  - 更新 `lastSeenTimestamp`。
  - 从 `removalPending` 中移除该目标（重置移除倒计时）。
  - 标记 `matchedTarget` 为本帧已匹配。

### 3. 处理未匹配的 `Target`

- 对未被匹配到的目标，调用 `target.markAllMembersMissed()`，将其下所有成员的 `consecutiveMisses++`，`consecutiveFrames` 置零。
- 将 `removalPending` 中该目标计数 +1。

### 4. 移除目标内过期成员

- 遍历 `targets`，调用 `target.removeStaleMembers(removalFrames, confirmationFrames)` 删除连续丢失帧数达标的成员。
- 若 `target.isEmpty()`，则将其在 `removalPending` 中的计数直接设为 `removalFrames`（加速移除）。

***

## 四、处理新候选目标（`handleNewCandidates()`）

### 1. 尝试将当前分组匹配现有候选（`CandidateTarget`）

- 与目标匹配逻辑类似：计算组中心，寻找距离最近且在阈值内的 `CandidateTarget`。
- 若匹配成功：
  - `consecutiveFrames++`，`consecutiveMisses` 置零。
  - 调用 `updateGroupAndMembers()` 更新内部成员。
  - 若 `consecutiveFrames ≥ confirmationFrames`，则**晋升为正式** **`Target`**，加入 `targets` 并标记待删除（从候选列表移除）。

### 2. 新建候选

- 若无匹配，则用当前分组创建 `CandidateTarget`，初始 `consecutiveFrames = 1`。

### 3. 候选老化处理

- 未被匹配的候选：`consecutiveMisses++`，`consecutiveFrames` 置零。
- 若 `consecutiveMisses ≥ removalFrames`，则从候选列表永久移除。

***

## 五、移除丢失的完整目标（`handleMissingTargets()`）

- 遍历 `removalPending`（记录每个 `Target` 连续未被匹配的帧数）。
- 若计数 ≥ `removalFrames` **且** 目标内已确认成员数为 0（`getActiveMemberCount(confirmationFrames) == 0`），则将该目标从 `targets` 和 `removalPending` 中彻底删除。

> 双重条件确保：只有目标既无活跃成员、又持续多帧未被观测到时才删除。

***

## 六、目标内部成员更新逻辑（`Target.updateGroupAndMembers()`）

### 1. 成员匹配（基于颜色与最近距离）

- 对当前组的每个 `Detection`，在目标现有成员中寻找**同颜色且距离 ≤ 0.3** 的最近成员。
- 若找到：
  - `consecutiveFrames++`，`consecutiveMisses` 置零，更新 `detection` 为当前值。
- 若未找到：
  - 创建新成员，`consecutiveFrames` 初始为 1。

### 2. 未匹配成员的衰减

- 遍历所有成员，未被匹配到的执行 `consecutiveMisses++`，`consecutiveFrames` 置零。

### 3. 更新目标中心坐标（`updateCenter()`）

- 仅使用 `consecutiveFrames ≥ confirmationFrames` 的成员坐标计算均值。
- 若无活跃成员，则中心置为 `(0, 0)`。

***

## 七、候选目标成员更新（`CandidateTarget.updateGroupAndMembers()`）

逻辑与 `Target` 类似，区别：

- 成员匹配距离阈值同样为 **0.3**。
- 更新中心时**使用所有成员坐标均值**（不区分确认状态），因为候选阶段尚未建立置信度。

***

## 八、最优目标选择（`getBestTarget()`）

1. 过滤：目标非空且活跃成员数 > 0。
2. 评分函数：
   ```java
   score = activeMemberCount / distanceToCamera
   ```
   其中 `distanceToCamera = sqrt(centerX² + centerY²)`，若为 0 则取 0.1 防止除零。
3. 返回得分最高的 `Target`（得分相同则取首个）。

> 该评分策略兼顾**目标包含的球数量多**和**距离近**两个因素。

***

## 九、关键数据结构与作用总结

| 类/结构              | 作用                                |
| :---------------- | :-------------------------------- |
| `Detection`       | 单帧检测结果（颜色 + 世界坐标）。                |
| `CandidateTarget` | 待确认的潜在目标，积累足够连续帧后晋升为 `Target`。    |
| `Target`          | 已确认的稳定目标，内部包含若干 `Member`。         |
| `Target.Member`   | 代表目标中一个具体颜色的球，维护连续出现/丢失计数器。       |
| `removalPending`  | 记录每个 `Target` 未被观测的连续帧数，用于整体移除判定。 |

***

## 十、滞后滤波（迟滞）机制示意

```
新球出现 → CandidateTarget（consecutiveFrames++）
          → 达到 confirmationFrames → 转为 Target.Member

球短暂消失 → consecutiveMisses++ 但未达阈值 → 仍保留在目标中
          → 达到 removalFrames → 从目标中移除

Target 所有成员丢失 → removalPending 计数累加
                   → 达到 removalFrames 且无活跃成员 → 删除 Target
```

该设计有效抑制了检测噪声与瞬时遮挡造成的抖动。
