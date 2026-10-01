完成Turret包负责控制炮台，要求如下：
炮台的结构：由一个大球发射器和一个小球发射器，和共同的平转机构组成，每个发射器含一个飞轮，一个由舵机控制的背板和一个舵机控制的扳机，两发射器的朝向始终平行，由一个电机控制。
各文件功能：
- Turret.java：负责炮台整体控制，提供一个update函数，每帧调用设置状态，参数如下：
  - yaw: 目标平转角度
  - pitchBig: 大球俯仰机构目标舵机位置（不一定是真实发射角度）
  - pitchSmall: 小球俯仰机构目标舵机位置（不一定是真实发射角度）
  - speedBig: 大球目标发射器速度
  - speedSmall: 小球目标发射器速度
  - triggerBig: 大球扳机是否释放
  - triggerSmall: 小球扳机是否释放
  返回两个bool变量，表示是否能够发射大球和小球
  超参数：
  - RelPosBig: 大球发射器中心在地面投影在相对坐标系的坐标，单位inch
  - RelPosSmall: 小球发射器中心在地面投影在相对坐标系的坐标，单位inch
  未来可实现自动瞄准
- YawController.java：负责平转机构的控制，提供一个update函数，每帧调用设置目标平转角度，钳制到预设范围内，计算电机目标位置，使用PIDSVA控制电机达到目标位置
    超参数：
  - yawRange: 平转机构最大角度绝对值，单位为弧度
  - tickPerRad: 每个弧度对应的电机tick变化量
  - yaw电机的PIDSVA参数
- Shooter.java：负责单个发射器的控制，提供一个update函数，每帧调用设置目标速度，舵机位置和扳机状态，钳制到可用范围后，使用PIDSVA控制电机达到目标速度，控制舵机达到目标角度。
    超参数：
  - shooter电机的PIDSVA参数
  - pitchMax: 舵机最大位置
  - pitchMin: 舵机最小位置
  - speedMax: 最大速度
  - TriggerOnPos: 扳机释放时目标舵机位置
  - TriggerOffPos: 扳机关闭时目标舵机位置
