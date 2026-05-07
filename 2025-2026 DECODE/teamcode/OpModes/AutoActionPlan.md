# AutoAction 自动模式逻辑说明

## 一、整体架构

`AutoAction` 是 FTC 机器人的自动模式 OpMode，实现了完整的自动游戏流程：搜索球 → 收集球 → 移动到射击区 → 射击。

## 二、初始化阶段

### 2.1 队伍颜色选择

```java
while (opModeInInit() || !initStarted) {
    if (gamepad1.a) teamColor = TEAM_COLOR.BLUE;
    if (gamepad1.b) teamColor = TEAM_COLOR.RED;
    
    switch (teamColor) {
        case BLUE: targetTagId = 20; break;
        case RED: targetTagId = 24; break;
    }
}
```

| 按键 | 功能 | 目标 AprilTag ID |
|------|------|----------------|
| A | 选择蓝队 | 20 |
| B | 选择红队 | 24 |

### 2.2 组件初始化

```java
ActionRunner actionRunner = new ActionRunner();
chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
turret = new Turret(hardwareMap, telemetry);
sweeper = new Sweeper(hardwareMap, telemetry);
tracker = new Tracker(hardwareMap);
tracker.start();
```

| 组件 | 作用 |
|------|------|
| ActionRunner | 管理动作队列，串行执行动作 |
| Chassis | 底盘控制，负责移动 |
| Turret | 炮塔控制，负责瞄准和发射 |
| Sweeper | 清扫器，收集球 |
| Tracker | 视觉追踪器，检测目标球 |

## 三、主循环逻辑

### 3.1 状态机流程图

```
┌─────────────────────────────────────────────────────────────────┐
│                    AutoAction 主循环                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                    while (opModeIsActive())             │    │
│  │  ┌─────────────────────────────────────────────────┐    │    │
│  │  │ RobotPosition.getInstance().update()            │    │    │
│  │  │ actionRunner.update()                           │    │    │
│  │  └─────────────────────────────────────────────────┘    │    │
│  │                         ↓                                 │    │
│  │              if (!actionRunner.isBusy())                   │    │
│  │                         ↓                                 │    │
│  │    ┌─────────────────┬─────────────────┬───────────────┐  │    │
│  │    ↓                 ↓                 ↓               │  │    │
│  │  [满仓 &&         [非空 &&          [空仓 ||           │  │    │
│  │   不在射击区]      在射击区]         (不满仓 &&        │  │    │
│  │                        │            不在射击区)]       │  │    │
│  │                        ↓                 ↓               │  │    │
│  │            GoToShootingAreaAction    ┌──────┴──────┐    │  │    │
│  │            ShootAction               ↓             ↓    │  │    │
│  │                              [检测到目标]    [未检测到]    │  │    │
│  │                                    ↓             ↓    │  │    │
│  │                              EatAction      SearchAction│  │    │
│  │                                                         │  │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 状态转换条件

| 条件 | 动作 | 说明 |
|------|------|------|
| `isFull() && !isAbleToShoot()` | GoToShootingAreaAction | 球仓已满但不在射击区，需要移动到射击区 |
| `!isEmpty() && isAbleToShoot()` | ShootAction | 有球且在射击区，执行射击 |
| `isEmpty() \|\| (!isFull() && !isAbleToShoot())` | EatAction / SearchAction | 需要收集球 |

### 3.3 动作执行与状态跟踪

使用 `lastActionType` 跟踪上一个执行的动作，实现有序的状态转换：

```java
private String lastActionType = "";

if(!actionRunner.isBusy()) {
    // 优先级1: EatAction 完成后直接进入 GoToShootingAreaAction
    if (lastActionType.equals("Eat")) {
        actionRunner.add(new GoToShootingAreaAction(chassis, sweeper));
        lastActionType = "GoToShootingArea";
    }
    // 优先级2: 满仓但不在射击区
    else if (RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot()) {
        actionRunner.add(new GoToShootingAreaAction(chassis, sweeper));
        lastActionType = "GoToShootingArea";
    }

    // 优先级3: 有球且在射击区
    if (!RobotPosition.getInstance().isEmpty() && RobotPosition.getInstance().isAbleToShoot()) {
        actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
        lastActionType = "Shoot";
    }

    // 优先级4: 空仓或不满仓且不在射击区
    if (RobotPosition.getInstance().isEmpty() || (!RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
        if(tracker.getBestTarget() != null){
            actionRunner.add(new EatAction(chassis, tracker, sweeper));
            lastActionType = "Eat";
        } else {
            actionRunner.add(new SearchAction(chassis, tracker, sweeper, teamColor));
            lastActionType = "Search";
        }
    }
}
```

### 3.4 状态转换流程

| 条件 | 动作 | 说明 |
|------|------|------|
| `lastActionType == "Eat"` | GoToShootingAreaAction | 吃球完成后直接去射击区 |
| `isFull() && !isAbleToShoot()` | GoToShootingAreaAction | 球仓已满但不在射击区 |
| `!isEmpty() && isAbleToShoot()` | ShootAction | 有球且在射击区，执行射击 |
| `isEmpty() \|\| (!isFull() && !isAbleToShoot())` | EatAction / SearchAction | 需要收集球 |

## 四、动作详解

### 4.1 GoToShootingAreaAction

将机器人移动到射击区域。

### 4.2 ShootAction

执行射击动作：
1. 使用 AprilTag 瞄准目标
2. 计算弹道参数
3. 发射球

### 4.3 EatAction

收集球动作：
1. 跟踪目标球
2. 移动到球的位置
3. 启动清扫器收集

### 4.4 SearchAction

搜索动作（关键逻辑）：

```java
public boolean run(TelemetryPacket packet) {
    tracker.update();

    // 如果检测到目标，提前中止轨迹
    if (tracker.getBestTarget() != null) {
        sweeper.setStop();
        return false;  // 提前中止，让 ActionRunner 调度 EatAction
    }

    // 根据队伍颜色选择搜索轨迹
    if (teamColor == Chassis.TEAM_COLOR.BLUE) {
        // 蓝队：下方搜索轨迹
        trajectoryAction = drive.actionBuilder(currentPose)
            .splineTo(new Vector2d(-15, -48), -Math.PI / 2)
            .lineTo(new Vector2d(-57, -48))
            .lineTo(new Vector2d(-15, -48))
            .build();
    } else {
        // 红队：上方搜索轨迹
        trajectoryAction = drive.actionBuilder(currentPose)
            .splineTo(new Vector2d(-15, 48), -Math.PI / 2)
            .lineTo(new Vector2d(-57, 48))
            .lineTo(new Vector2d(-15, 48))
            .build();
    }

    // 执行搜索轨迹
    sweeper.setEat();
    // ... 沿预定轨迹移动
}
```

**搜索轨迹（按队伍颜色）**：

| 队伍颜色 | Y坐标 | 搜索路径 |
|----------|-------|----------|
| **蓝队 (BLUE)** | -48 | (-15,-48) → (-57,-48) → (-15,-48) |
| **红队 (RED)** | +48 | (-15,48) → (-57,48) → (-15,48) |

**提前中止机制**：在搜索过程中，如果 `tracker` 检测到目标球，立即停止轨迹，让 `ActionRunner` 切换到 `EatAction`。

## 五、关键设计特点

### 5.1 动作队列机制

使用 `ActionRunner` 管理动作队列，实现：
- 动作串行执行
- 非阻塞更新
- 动作优先级管理

### 5.2 状态驱动

基于机器人状态（球仓状态、位置状态）决定下一步动作，实现智能决策。

### 5.3 视觉反馈

`Tracker` 实时更新目标检测状态，支持：
- 目标颜色识别（紫色/绿色）
- 低通滤波平滑
- 距离得分排序

### 5.4 动态切换

`SearchAction` 支持中途切换到 `EatAction`，提高响应速度。

## 六、代码优化建议

### 6.1 状态机重构

当前使用多个 if 语句判断状态，可以考虑引入状态机模式：

```java
enum RobotState {
    SEARCHING, EATING, GOING_TO_SHOOTING_AREA, SHOOTING
}
```

### 6.2 动作优先级

当前多个条件可能同时满足（如满仓且在射击区），建议使用 if-else 结构确保唯一动作选择。

### 6.3 日志增强

增加关键状态的日志输出，便于调试和问题定位。
