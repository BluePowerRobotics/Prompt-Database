Action列表：
GoToShootingArea: 调用Chassis类的GoToShootingArea()
开始条件：RobotPosition.isFull()==true
结束条件：RobotPosition.ableToShoot==true
Stop: 调用Chassis类的stop()
Shoot: 调用Turret类发射小球
Score=ParallelAction(Shoot,Stop)
开始条件：RobotPosition.ableToShoot==true
结束条件：RobotPosition.isEmpty()==true
Search: 调用Chassis类的GoTo()前往Tracker类返回的目标角度
开始条件：RobotPosition.isEmpty()==true
结束条件：RobotPosition.isFull()==true
