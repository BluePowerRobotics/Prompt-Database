现在要在java实现ConvexPolygon类，用于描述凸多边形，并利用其性质实现一些函数
每个凸多边形通过顶点定义，每个顶点由Point2D类表示，顶点总数为n，n>=3(否则throwException)。
函数包括：
- 构造函数：传入所有顶点，按顶点逆时针顺序排序
- isConvex()：返回是否为凸多边形（在构造函数中判断，若不是则throwException）
- inRelative(x, y, theta):输出一个凸多边形，表示当前凸多边形由绝对坐标系转到相对坐标系后的凸多边形(先平移x,y后旋转theta角度)
- inRelative(Point2D p):输出一个凸多边形，表示当前凸多边形由绝对坐标系转到相对坐标系后的凸多边形(先平移p后旋转theta角度)
- inAbsolute(x, y, theta):输出一个凸多边形，表示当前凸多边形由相对坐标系转到绝对坐标系后的凸多边形（先旋转-theta角度后平移-x,-y）
- inAbsolute(Point2D p):输出一个凸多边形，表示当前凸多边形由相对坐标系转到绝对坐标系后的凸多边形（先旋转-theta角度后平移-x,-y）
- NearestVectorFrom(x, y):输出两个数表示一个向量，表示由(x,y)指向当前凸多边形中离(x,y)最近的点的向量
- NearestVectorFrom(Point2D p):输出两个数表示一个向量，表示由p点指向当前凸多边形中离p最近的点的向量
- Contains(x, y):返回(x,y)是否在当前凸多边形内（对严格逆时针凸多边形，点在内部（含边界）的充要条件是：对于每条边Vi Vi+1，点P均在边的左侧，即叉积Vi Vi+1*Vi P≥0（允许数值误差））。
- Contains(Point2D p):返回p是否在当前凸多边形内
- Contains(ConvexPolygon convexpolygon):返回convexpolygon是否在当前凸多边形内(包含边界，方法是判断convexpolygon的所有点是否都在当前凸多边形内)
- IntersectWith(ConvexPolygon convexpolygon):输出一个凸多边形，表示当前凸多边形与convexpolygon的交集（采用 Sutherland‑Hodgman 多边形裁剪算法：用裁剪多边形（clip）的每条边作为裁剪线，依次裁剪被裁剪多边形（subject）。因两个多边形都是凸的，交集仍为凸多边形。处理完所有裁剪边后，移除生成顶点序列中的共线点，得到严格凸的交集。如果交集为空、退化为点或线段（最终顶点数 <3），抛出异常。）