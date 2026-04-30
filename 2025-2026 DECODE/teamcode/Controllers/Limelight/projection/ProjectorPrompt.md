# 计算原理说明
(x_ball, y_ball)表示小球在画面坐标

(x0, y0)表示画面中心点坐标

M表示画面长边长度(常量)

m=(x_ball-x0)/M表示横向偏移

n=(y_ball-y0)/M表示纵向偏移

h表示摄像头高度

m0表示摄像头画面中一米远处一米长的物体在画面中对应的长度与画面长边长的比值(常量)

rt2表示根号2

d=(rt2*m0*h)/(m0-n)表示像距

(delta_x, delta_y)表示小球相对摄像头的位置

delta_x=rt2*d-h

delta_y=m*d/m0

theta表示小车方向与x轴夹角（逆时针为正）

phi表示摄像头方向与小车方向夹角（逆时针为正）(常量)

(x_robot, y_robot)表示小车在世界坐标中的位置

(x, y)表示小球在世界坐标中的位置

(x,y)为(x_robot, y_robot)加上(delta_x, delta_y)逆时针旋转(theta+phi)角度后的和

# 提示词
请根据以上公式，在projector.java中实现project函数，输入为(x_ball, y_ball)和(x_robot, y_robot)和(theta, phi)，设m0已知为1，输出世界坐标中的(x, y)
