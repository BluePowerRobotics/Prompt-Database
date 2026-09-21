# 官方 ColorLocator（颜色定位）使用说明

ColorLocator 基于 FTC SDK 内置的 `ColorBlobLocatorProcessor`（以下简称 CBLP），它由
`VisionPortal` 驱动摄像头，在画面中查找符合指定颜色范围的"色块（blob）"，并返回每个色块的
轮廓、拟合矩形/圆、面积、圆形度等几何信息。

> 与"颜色传感器（ColorSensor）"不同：ColorSensor 只能判断贴近传感器的物体颜色，
> ColorLocator 则是在整幅（或局部）画面里**定位**目标色块的位置与形状。

参考官方样例：

- `ConceptVisionColorLocator_Rectangle.java`（矩形拟合）
- `ConceptVisionColorLocator_Circle.java`（圆形拟合）

***

## 一、主要依赖（import）

```java
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.Circle;        // 圆形拟合（Circle 样例）
import org.firstinspires.ftc.vision.opencv.ColorSpace;     // 自定义色范围时使用

import org.opencv.core.RotatedRect;  // 矩形拟合
import org.opencv.core.Scalar;       // 自定义色范围时使用

import java.util.List;
```

***

## 二、工作流程

1. **构建处理器**：用 `ColorBlobLocatorProcessor.Builder` 配置目标色范围、ROI、轮廓模式、
   预处理（模糊/腐蚀/膨胀）等，`build()` 得到 `colorLocator`。
2. **构建 VisionPortal**：把 `colorLocator` 用 `addProcessor` 挂载，并设定分辨率与摄像头。
3. **读取结果**：循环里调用 `colorLocator.getBlobs()` 得到 `List<Blob>`。
4. **过滤/排序**：用 `ColorBlobLocatorProcessor.Util` 按面积、圆形度等筛选、排序。
5. 每个 `Blob` 可取出中心坐标、面积、密度、长宽比、周长、圆形度，以及矩形框/圆框。

算法内部：匹配 `ColorRange` 的像素形成二值 `mask` → 聚成连通的 `blob` → 提取轮廓
`contour` → 为每个 blob 生成能完全包裹轮廓的最小矩形 `boxFit`（或最小圆）。

***

## 三、构建处理器与参数详解

```java
ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // 目标色范围（必选）
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        .setDrawContours(true)                              // 预览层是否画轮廓
        .setBoxFitColor(0)                                  // 矩形框颜色，0 关闭
        .setCircleFitColor(Color.rgb(255, 255, 0))          // 圆框颜色，0 关闭
        .setBlurSize(5)                                     // 模糊
        .setDilateSize(15)                                  // 膨胀
        .setErodeSize(15)                                   // 腐蚀
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
        .build();
```

### 3.1 setTargetColorRange(ColorRange) —— 目标色范围

**预定义颜色**（最简单）：

| 常量                                             | 说明    |
| ---------------------------------------------- | ----- |
| `ColorRange.RED` / `BLUE` / `YELLOW` / `GREEN` | 基础色   |
| `ColorRange.ARTIFACT_GREEN`                    | 目标元素绿 |
| `ColorRange.ARTIFACT_PURPLE`                   | 目标元素紫 |

**自定义色范围**（在 `YCrCb` 等色彩空间里给上下界）：

```java
.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
        new Scalar( 32, 176,   0),   // 下界
        new Scalar(255, 255, 132)))  // 上界
```

### 3.1.1 官方支持的色彩空间

官方 `org.firstinspires.ftc.vision.opencv.ColorSpace` 枚举只提供以下**三种**色彩空间，
自定义 `ColorRange` 时用它来声明上下界数值所属的空间：

| 枚举值     | 含义           | OpenCV 转换码（BGR → 该空间）     | 各通道范围（8 位图）                          |
| ------- | ------------ | ------------------------- | ------------------------------------ |
| `RGB`   | 红-绿-蓝        | `Imgproc.COLOR_BGR2RGB`   | R / G / B：0 \~ 255                   |
| `HSV`   | 色相-饱和度-明度    | `Imgproc.COLOR_BGR2HSV`   | H：0 \~ 180（见下文），S / V：0 \~ 255       |
| `YCrCb` | 亮度-红色色差-蓝色色差 | `Imgproc.COLOR_BGR2YCrCb` | Y / Cr / Cb：0 \~ 255（Cr、Cb 中心约为 128） |

> 几点说明：
>
> - 官方 `ColorLocator` / `ColorBlobLocatorProcessor` **不支持 Lab、HSL/HLS 等空间**。
> - OpenCV 中 HSV 的 **H 通道实际为 0 \~ 179**（官方样例注释里写作 0 \~ 180），
>   而 S、V 均为 0 \~ 255。
> - 从样例源码可看到，预定义颜色常量（`ColorRange.RED` / `BLUE` / `YELLOW` / `GREEN` /
>   `ARTIFACT_GREEN` / `ARTIFACT_PURPLE`）内部均使用 `YCrCb` 空间标定的上下界。

### 3.2 setRoi(ImageRegion) —— 感兴趣区域（可选）

只在一部分画面里搜索，可减小干扰、提高速度。`ImageRegion` 三种写法：

```java
ImageRegion.entireFrame();                                    // 整帧
ImageRegion.asImageCoordinates(50, 50, 150, 150);             // 像素坐标：左上角 100x100
ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5);   // 归一化坐标（-1..1，中心为原点）
```

> 注意 `asUnityCenterCoordinates(uMin, vMin, uMax, vMax)` 中 v 方向与图像 y 相反，
> 上方为正值；样例 `(-0.75, 0.75, 0.75, -0.75)` 表示画面中心 75% 宽高区域。

### 3.3 setContourMode(ContourMode) —— 轮廓模式

| 模式                        | 说明                               |
| ------------------------- | -------------------------------- |
| `EXTERNAL_ONLY`           | 只保留最外层轮廓，**推荐**，可避免亮斑反射把实心色块切成碎片 |
| `ALL_FLATTENED_HIERARCHY` | 保留所有轮廓（含被内部轮廓）                   |

### 3.4 预处理（可选的形态学/降噪）

| 方法                           | 说明                                                       |
| ---------------------------- | -------------------------------------------------------- |
| `setBlurSize(int px)`        | 高斯模糊，平滑颜色过渡与轮廓；数值越大越模糊，会隐藏小特征。低分辨率图建议 5。偶数会自动 +1 满足奇数要求  |
| `setErodeSize(int px)`       | 腐蚀：去除孤立像素和细线，会缩小物体、可能放大内部孔洞。低分辨率建议 2\~4                  |
| `setDilateSize(int px)`      | 膨胀：填小孔、连接断裂部分、让物体变大。低分辨率建议 2\~4                          |
| `setMorphOperationType(...)` | 指定腐蚀/膨胀顺序：`OPENING`=先腐蚀后膨胀（去小噪点）；`CLOSING`=先膨胀后腐蚀（填边缘缺口） |

### 3.5 预览层绘制（调试用，会消耗 CPU）

| 方法                             | 说明                                     |
| ------------------------------ | -------------------------------------- |
| `setDrawContours(boolean)`     | 是否在预览上画轮廓                              |
| `setBoxFitColor(int color)`    | 矩形框颜色（`android.graphics.Color`），`0` 关闭 |
| `setCircleFitColor(int color)` | 圆框颜色，`0` 关闭                            |

***

## 四、构建 VisionPortal

```java
VisionPortal portal = new VisionPortal.Builder()
        .addProcessor(colorLocator)
        .setCameraResolution(new Size(320, 240))   // 该任务低分辨率即可，能提升性能、降低延迟
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // 或用 BuiltinCameraDirection.BACK
        .build();
```

***

## 五、读取结果：Blob 对象

```java
List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
```

默认按轮廓面积**降序**返回（最大色块在最前）。每个 `Blob` 提供：

| 方法                 | 返回            | 说明                                       |
| ------------------ | ------------- | ---------------------------------------- |
| `getBoxFit()`      | `RotatedRect` | 包裹轮廓的最小旋转矩形；`boxFit.center.x/.y` 为中心像素坐标 |
| `getCircle()`      | `Circle`      | 包裹轮廓的最小圆；`getX()/getY()/getRadius()`     |
| `getContourArea()` | int           | 轮廓内像素面积                                  |
| `getDensity()`     | double        | 填充程度 = 轮廓面积 / 凸包面积，越接近 1 越"实心"           |
| `getAspectRatio()` | double        | 框长边/短边，正方形≈1                             |
| `getArcLength()`   | double        | 轮廓周长                                     |
| `getCircularity()` | double        | 圆形度，正圆≈1，越小越不圆                           |

***

## 六、过滤与排序

```java
// 过滤：只保留面积 50~20000 的色块
ColorBlobLocatorProcessor.Util.filterByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);

// 过滤：只保留圆形度 0.6~1 的色块（圆形目标）
ColorBlobLocatorProcessor.Util.filterByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

// 排序（最多调用一次）
ColorBlobLocatorProcessor.Util.sortByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
```

可用的 `BlobCriteria`：

| 枚举                | 说明  |
| ----------------- | --- |
| `BY_CONTOUR_AREA` | 面积  |
| `BY_DENSITY`      | 密度  |
| `BY_ASPECT_RATIO` | 长宽比 |
| `BY_ARC_LENGTH`   | 周长  |
| `BY_CIRCULARITY`  | 圆形度 |

> `filterByCriteria` 只影响返回的 `blobs` 列表，不会影响预览层上绘制的轮廓。

***

## 七、多颜色查找（同一 VisionPortal 挂多个处理器）

`setTargetColorRange(ColorRange)` 每次只接受**一个**颜色范围，单个 `ColorBlobLocatorProcessor`
无法直接表达"多颜色取并"。若要同时查找多种颜色（例如红色和蓝色），做法是：**为每种颜色各建一个
`ColorBlobLocatorProcessor`，把这多个处理器都** **`addProcessor(...)`** **到同一个** **`VisionPortal`，
再手动合并它们的** **`getBlobs()`** **结果**。

好处：每个处理器可拥有独立的 `setRoi` / `setContourMode` / 预处理参数，覆盖区域可以重叠。

```java
import java.util.ArrayList;
import java.util.List;

// 为每种颜色各建一个处理器
ColorBlobLocatorProcessor redLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.RED)
        .setRoi(ImageRegion.entireFrame())
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .build();

ColorBlobLocatorProcessor blueLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.BLUE)
        .setRoi(ImageRegion.entireFrame())
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .build();

// 挂到同一个 VisionPortal
VisionPortal portal = new VisionPortal.Builder()
        .addProcessor(redLocator)
        .addProcessor(blueLocator)
        .setCameraResolution(new Size(320, 240))
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .build();

while (opModeIsActive() || opModeInInit()) {
    // 取并：合并各处理器的结果
    List<ColorBlobLocatorProcessor.Blob> all = new ArrayList<>();
    all.addAll(redLocator.getBlobs());
    all.addAll(blueLocator.getBlobs());

    // 合并后再统一过滤/排序
    ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, all);
    ColorBlobLocatorProcessor.Util.sortByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, all);

    for (ColorBlobLocatorProcessor.Blob b : all) {
        RotatedRect box = b.getBoxFit();
        telemetry.addData("Center", "(%3.0f, %3.0f)", box.center.x, box.center.y);
    }
    telemetry.update();
    sleep(100);
}
```

补充：

- 若只是"同一色相、跨 0° 环绕"的宽范围（例如 HSV 红色 H ∈ \[170,180] ∪ \[0,10]），
  `ColorRange` 的单一矩形区间无法覆盖，同样可用本方案建两个处理器；
- 若需要更灵活的"任意多区间取并"（如自定义色彩空间、多个上下界一次性合并），可改用自定义
  `VisionProcessor`，直接对 mask 做多区间 `inRange` + `bitwise_or`（参考本项目的 `ColorSegCam`）。

### 7.1 合并不同颜色的 blob（把多色拼接的同一物体当作一个 blob）

若目标是"一个由红、绿等**多种颜色拼接**成的物体，只识别成一个 blob"，官方 CBLP 无法在
像素层面直接合并：每个处理器阈值独立、`Blob` 又不暴露原始 mask/轮廓，两种颜色只会产出
两个互不相干的 blob。要合并成一个，有两条路：

**方法一：几何近似合并（仅依赖官方 `Blob` 的几何信息）**

对第七节拼接出的 `all` 列表，按"外接框是否重叠/邻近"把相邻 blob 合并：重心按面积加权、
面积相加、外接框取并集。

```java
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

// 简易"复合色块"，承载合并后的几何信息
static class MergedBlob {
    Rect bounds;   // 外接框（AABB 近似）
    double cx, cy; // 面积加权重心（像素坐标）
    int area;      // 面积之和
}

List<MergedBlob> merged = new ArrayList<>();
for (ColorBlobLocatorProcessor.Blob b : all) {
    RotatedRect box = b.getBoxFit();
    Rect r = box.boundingRect();
    MergedBlob hit = null;
    for (MergedBlob m : merged) {
        // 两外接框是否重叠，或间隙 <= 10 像素
        boolean overlap = m.bounds.x < r.x + r.width + 10 && r.x < m.bounds.x + m.bounds.width + 10
                       && m.bounds.y < r.y + r.height + 10 && r.y < m.bounds.y + m.bounds.height + 10;
        if (overlap) { hit = m; break; }
    }
    if (hit == null) {
        MergedBlob m = new MergedBlob();
        m.bounds = new Rect(r.x, r.y, r.width, r.height);
        m.cx = box.center.x;
        m.cy = box.center.y;
        m.area = b.getContourArea();
        merged.add(m);
    } else {
        int a = b.getContourArea();
        double na = hit.area + a;                                    // 面积相加
        hit.cx = (hit.cx * hit.area + box.center.x * a) / na;        // 面积加权重心
        hit.cy = (hit.cy * hit.area + box.center.y * a) / na;
        hit.area = (int) na;
        int x1 = Math.min(hit.bounds.x, r.x);                        // 外接框取并集
        int y1 = Math.min(hit.bounds.y, r.y);
        int x2 = Math.max(hit.bounds.x + hit.bounds.width,  r.x + r.width);
        int y2 = Math.max(hit.bounds.y + hit.bounds.height, r.y + r.height);
        hit.bounds = new Rect(x1, y1, x2 - x1, y2 - y1);
    }
}
```

> 局限：这纯粹靠几何邻近度合并，不感知真实像素是否连通。两个**同色但恰好贴在一起**的物体
> 也会被误合并；只有**异色且物理相连**的物体才能正确合并，适合对精度要求不高的场景。

**方法二：像素级合并（自定义 `VisionProcessor`，推荐）**

要真正让"异色相邻像素连成同一个 blob"，必须让所有目标颜色落在**同一张 mask** 里：对同一帧
做多个 `inRange` 区间，再用 `bitwise_or` 合并成一张 mask，然后跑连通域/找轮廓。这正是本项目
`ColorSegCam` 的实现思路；官方 CBLP 每个处理器各有一张独立 mask，无法做到这一点。

***

## 八、完整代码示例


```java
        // ===== 需要的 import（按项目模板补充）=====
        // import java.util.ArrayList;
        // import java.util.List;
        // import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        // import org.firstinspires.ftc.robotcore.external.navigation.Size;
        // import org.firstinspires.ftc.robotcore.external.tfod.SortOrder;
        // import org.firstinspires.ftc.vision.VisionPortal;
        // import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
        // import org.firstinspires.ftc.vision.opencv.ColorRange;
        // import org.firstinspires.ftc.vision.opencv.ColorSpace;
        // import org.firstinspires.ftc.vision.opencv.ImageRegion;
        // import org.opencv.core.RotatedRect;

        // ===== 初始化阶段 =====  （放入 runOpMode 开头）
        // 构建每个 ColorBlobLocatorProcessor
        ColorBlobLocatorProcessor proc0 = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                                new Scalar(0, 0, 0),
                                new Scalar(255, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(5)
                .setErodeSize(0)
                .setDilateSize(0)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // 挂载到同一个 VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(proc0)
                .setCameraResolution(new Size(640, 360))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // 或用 BuiltinCameraDirection.BACK
                .build();

        telemetry.setMsTransmissionInterval(100);

        // ===== 运行阶段 =====  （放入 runOpMode 循环体内）
        // 读取每个 Processor 的色块列表
        List<ColorBlobLocatorProcessor.Blob> blobs0 = proc0.getBlobs();

        // 合并所有色块
        List<ColorBlobLocatorProcessor.Blob> allBlobs = new ArrayList<>();
        allBlobs.addAll(blobs0);

        // 全局排序
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, allBlobs);

        // 只提取排序第一的色块，并存储其拟合形状与关键参数
        if (!allBlobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob firstBlob = allBlobs.get(0);
            ColorBlobLocatorProcessor.Circle circle = firstBlob.getCircle();
            double circleCenterX = circle.getX();
            double circleCenterY = circle.getY();
            double circleRadius = circle.getRadius();
        }

```

***

## 九、调试与性能提示

- **实时预览**：把 `while (opModeIsActive())` 写成 `while (opModeIsActive() || opModeInInit())`，
  即可在 INIT 阶段看到摄像头流与色块框，方便现场标定参数。
- **查看画面**：可在 Control Hub 接 HDMI 显示器，或用 ScrCpy 远投。
- **分辨率**：ColorLocator 对分辨率不敏感，用 `320x240` 即可，省 CPU、降延迟。
- **圆形度受光照影响**：阴影会改变 `circularity`，若规则允许标定，建议在赛场上实测阈值。
- **CPU 开销**：`setDrawContours`/`setBoxFitColor`/`setCircleFitColor` 的绘制会占用 CPU，
  正式比赛算法里可在调试完成后关闭。

