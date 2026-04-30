完成Turret类，封装所有炮台控制函数，主要参数包括：

roll: 表示绕z轴角，即当前与x轴正方向（小车正前方）夹角（逆时针为正）

yaw：表示绕y轴角，即仰角

k, b: 待测的转换参数

主要函数包括：

rotate\_to(roll, yaw) 旋转到与小车正前方逆时针夹角为roll，仰角yaw的位置，完成后返回true，失败返回false

get\_angle() 输出当前roll和yaw

aim() 用ApriltagProcessor及ApriltagPoseFtc库实现，从webcam获取当前apriltag的角度偏移，若检测到则调用rotate\_to函数修正，否则用rotate\_to水平逆时针旋转90度，反复循环直到角度偏移<0.5度；用get\_angle函数获取当前仰角和旋转角（即目标的仰角和旋转角）输出

set(k, b) 设置k和b

shoot(roll, yaw)，输入目标的仰角和旋转角，计算出目标的相对位置，随后调用utility.RK4计算发射参数（初始值为输入的角度），将v0转换为转速（n=kv0+b，k.b通过线性回归确定），发射小球

update() 用于每帧调用，输入一个bool变量表示是否发射，调用aim瞄准目标，如果输入为真则调用shoot发射
