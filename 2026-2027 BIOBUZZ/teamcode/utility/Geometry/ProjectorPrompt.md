在EulerAngle.java中实现基于欧拉角的三维姿态类，有三个属性：roll, pitch, yaw，分别表示绕x轴、y轴、z轴的旋转角度，变换顺序ZYX，并提供坐标系变换方法transform(Vector3d vec) 输出一个Vector3d类型的向量，表示变换后坐标系中的坐标, 和reform(Vector3d vec) 输出一个Vector3d类型的向量，表示变换前坐标系中的坐标。
完成后根据Plan.md实现Projector类，有一属性EulerAngle Pose，用于表示当前投影器的姿态,另有一属性l0。提供一个函数project(x_p, y_p, k) 输出一个Vector3d类型的向量，表示绝对坐标
