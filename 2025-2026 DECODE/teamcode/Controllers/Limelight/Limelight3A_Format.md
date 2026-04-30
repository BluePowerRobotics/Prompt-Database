以下是 Limelight 3A 神经检测器（Neural Detector）输出格式的官方详细说明。你需要理解这些数据，才能在 FTC 机器人的代码中准确获取每个被检测物体的位置和类别信息。

***

### 🔎 神经检测器（Neural Detector）输出格式

Limelight 的神经检测器是专门为“目标检测”任务设计的，它能识别出图像中特定对象（如游戏道具）的位置，并返回它们的 **边界框 (Bounding Box)** 和 **类别 (Class)** 信息。

官方推荐使用 JSON 格式来获取和解析这些数据，因为它具有人类可读性，并且在任何平台和任何编程语言中都易于解析。这些 JSON 数据可以通过多种接口获取，包括 REST/HTTP API、Websocket 和 NetworkTables。在 FTC 中，最直接的方式是使用官方提供的 Java API（`Limelight3A` 类）。

***

#### 1. JSON 输出结构

当 Limelight 3A 运行神经检测器管道时，其输出的 JSON 结果中会包含一个名为 **`Detector`** 的数组。这个数组是 JSON 对象的一部分，整个 JSON 对象可能同时包含其他类型管道（如 AprilTag）的结果。

一个典型的包含神经检测器结果的 JSON 对象结构如下：

```json
{
  // ... 其他全局字段 (如 v, ts, tl 等) ...
  "Detector": [
    {
      "class": "person",
      "classID": 0,
      "conf": 0.83984375,
      "ta": 0.2608712911605835,
      "tx": -2.45949649810791,
      "txp": 147.5,
      "ty": -10.066887855529785,
      "typ": 165.5,
      "pts": []
    },
    // ... 可能还有更多检测结果 ...
  ],
  // ... 其他数组 (如 Fiducial, Retro 等) ...
}
```

`Detector` 数组中的每个元素都代表了一个被检测到的物体，其内部字段含义如下表所示：

| 字段名           | 值类型     | 描述                                |
| :------------ | :------ | :-------------------------------- |
| **`class`**   | 字符串     | 目标的人类可读的类别名称，例如 `"person"`。       |
| **`classID`** | 整数      | 目标的类别ID（整数值）。                     |
| **`conf`**    | 浮点数     | 检测的置信度，范围是 0 到 1。                 |
| **`ta`**      | 浮点数     | 目标大小，以占整个图像面积的百分比表示，范围是 0 到 1。    |
| **`tx`**      | 浮点数（度）  | 目标中心点相对于十字准星的水平偏移角度。正值表示在右侧。      |
| **`ty`**      | 浮点数（度）  | 目标中心点相对于十字准星的垂直偏移角度。正值表示在下方。      |
| **`txp`**     | 浮点数（像素） | 目标中心点相对于十字准星的水平像素偏移。              |
| **`typ`**     | 浮点数（像素） | 目标中心点相对于十字准星的垂直像素偏移。              |
| **`pts`**     | 数组      | 目标的四个角点（如果启用）。每个角点是 `[x, y]` 坐标对。 |

## **特别注意：由于win和linux的换行符不同，limelight无法自动去除名称后的\r字符，请在代码中使用`if(className.equals(objectName)||className.equals(objectName + "\r"))`**

> **请注意**：虽然表格中提到了 `pts`，但通常你可能只需要使用 `tx` 和 `ty` 来定位，或用 `ta` 来判断大致距离。你可以通过在Limelight Web界面“Output”选项卡的“Detector Corner Points”中启用输出设置来开启 `pts` 数据。

***

#### 2. 在 FTC Java 代码中读取输出

在 FTC 的 Java 代码中，你不需要手动解析 JSON，因为 `Limelight3A` 类已经为你封装好了。获取神经检测器结果的核心步骤如下：

1. **获取** **`LLResult`** **对象**：通过 `limelight.getLatestResult()` 获取最新的处理结果。
2. **检查结果有效性**：调用 `result.isValid()` 确保结果是有效的。
3. **获取检测结果列表**：使用 `result.getDetectorResults()` 获取一个 `List<DetectorResult>` 列表，其中包含了当前帧所有检测到的物体信息。
4. **遍历并读取数据**：遍历这个列表，从每个 `DetectorResult` 对象中获取你需要的数据。

以下是一个完整的代码示例：

```java
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
// ... 其他 imports

public void processDetections(Limelight3A limelight, Telemetry telemetry) {
    // 获取最新的检测结果
    LLResult result = limelight.getLatestResult();

    // 检查结果是否有效（tv字段为1）
    if (result != null && result.isValid()) {
        // 获取所有检测器结果（物体列表）
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // 检查是否检测到任何物体
        if (detections.isEmpty()) {
            telemetry.addData("Detection Status", "No objects detected");
            return;
        }

        telemetry.addData("Detection Status", "Objects found: " + detections.size());

        // 遍历每一个检测到的物体
        for (LLResultTypes.DetectorResult detection : detections) {
            String className = detection.getClassName();   // 物体类别名称
            double confidence = detection.getConfidence(); // 置信度（0-1）
            double x = detection.getTargetXDegrees();      // 水平偏移（度）
            double y = detection.getTargetYDegrees();      // 垂直偏移（度）
            double area = detection.getTargetArea();       // 目标面积（占图像比例 0-1）

            telemetry.addData("Object", String.format("Class: %s, Conf: %.2f, X: %.2f, Y: %.2f, Area: %.2f",
                                                      className, confidence, x, y, area));
        }
    } else {
        telemetry.addData("Limelight", "Invalid result or no targets");
    }
}
```

***

#### 3. 全局状态信息（`v` 字段）

除了 `Detector` 数组，JSON 结果中还包含一个全局的 `v` 字段，代表 **有效性指示器 (Validity Indicator)**。它的值是 1 或 0：

- **`v = 1`**：表示当前帧存在有效目标。
- **`v = 0`**：表示当前帧不存在有效目标。

在 FTC Java API 中，这对应 `result.isValid()` 方法，如上例所示。在手动解析 JSON 时，你应当首先检查 `v` 字段的值。

***

#### 4. 多目标检测

官方文档明确指出，Limelight 的输出支持多个目标，这在 `Detector` 数组中体现为多个对象。FTC API 中的 `List<DetectorResult>` 也正是为处理多个检测结果而设计的。

***

### ⚙️ 关键设置与注意事项

1. **启用神经检测器管道 (Pipeline Type)**：确保你在 Limelight 的 Web 界面中将“Pipeline Type”设置为 **“Neural Detector”**。
2. **上传你的自定义模型 (.tflite)**：通过 Limelight 的 Web 界面上传你的训练好的 `.tflite` 模型文件，并确保正确配置。
3. **CPU 推理 (Inference Engine)**：Limelight 3A **不支持** Google Coral 加速器，神经网络推理完全在设备自身的 **CPU** 上进行。这意味着你的神经网络模型应当针对 CPU 性能进行优化。
4. **置信度阈值 (Confidence Threshold)**：你可以通过 Web 界面调整“confidence threshold”滑块来过滤掉低置信度的检测结果。你也可以在代码中通过检查 `conf` 字段或 `detection.getConfidence()` 的值，根据你的需求进行二次过滤。

