以下是 Limelight 3A 上传 AprilTag Map 的具体接口实现方案，涵盖 REST API 规范、Java 代码示例以及与 HIVE 状态机的集成方式。

***

## 一、接口规范

Limelight OS 在 `(limelight-ip):5807` 上运行 REST/HTTP 服务器，上传场地地图的接口为：

| 项目         | 内容                                              |
| :--------- | :---------------------------------------------- |
| **方法**     | `POST`                                          |
| **路由**     | `/upload-fieldmap`                              |
| **请求体**    | JSON 格式的 `.fmap` 数据                             |
| **可选参数**   | URL 参数 `index`（指定加载到哪个 pipeline，默认为当前 pipeline） |
| **完整 URL** | `http://<limelight-ip>:5807/upload-fieldmap`    |

上传后地图会覆盖磁盘上的场地地图，Limelight 随即使用新地图进行 MegaTag 位姿解算。

### .fmap 文件格式

`.fmap` 是一个 JSON 文件，包含一个 `fiducials` 数组，每个条目描述一个 AprilTag：

```json
{
  "type": "ftc",
  "fiducials": [
    {
      "family": "36h11",
      "id": 30,
      "size": 82.5,
      "transform": [
        1, 0, 0, -36.0,
        0, 1, 0,  22.0,
        0, 0, 1,  -15.0,
        0, 0, 0,   1.0
      ],
      "unique": 1
    }
  ]
}
```

| 字段          | 说明                                      |
| :---------- | :-------------------------------------- |
| `type`      | 场地类型，FTC 使用 `"ftc"` 或 `"ftcd"`（菱形场地）    |
| `family`    | AprilTag 家族，BIOBUZZ 使用 `36h11`          |
| `id`        | 标签 ID（30–45）                            |
| `size`      | 标签尺寸（毫米），BIOBUZZ 为 82.5 mm（3.25 in）     |
| `transform` | 4×4 行优先变换矩阵，SI 单位（米），表示该 Tag 在场地坐标系中的位姿 |
| `unique`    | 是否唯一（1 = 该 ID 仅出现一次）                    |

你需要在赛前为 HIVE 的两个稳定状态分别生成两份 `.fmap`。每份只包含当前**朝上 CELL** 的 4 个 Tag。transform 矩阵需要根据 BIOBUZZ 官方提供的 HIVE 几何尺寸和两个稳定状态下的 Tag 位置计算得到。

## 二、Java 上传实现

FTC 的 Android 环境中没有 `OkHttp` 等第三方 HTTP 库的默认依赖，使用 JDK 自带的 `HttpURLConnection` 即可。以下是一个可直接集成到 `MT1Localizer` 中的工具类：

```java
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

public class LimelightMapUploader {

    /** Limelight 默认 IP（通过 USB 连接时） */
    private static final String LIMELIGHT_IP = "172.28.0.1";
    private static final int PORT = 5807;

    /**
     * 上传 .fmap 到 Limelight。
     *
     * @param fmapJson  .fmap 文件的 JSON 字符串内容
     * @param pipelineIndex  目标 pipeline 索引（-1 表示使用当前 pipeline）
     * @return true 如果上传成功（HTTP 200）
     */
    public static boolean uploadFieldMap(String fmapJson, int pipelineIndex) {
        HttpURLConnection conn = null;
        try {
            String urlStr = "http://" + LIMELIGHT_IP + ":" + PORT + "/upload-fieldmap";
            if (pipelineIndex >= 0) {
                urlStr += "?index=" + pipelineIndex;
            }

            URL url = new URL(urlStr);
            conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setDoOutput(true);
            conn.setConnectTimeout(2000);
            conn.setReadTimeout(3000);

            // 写入 JSON 请求体
            try (OutputStream os = conn.getOutputStream()) {
                byte[] input = fmapJson.getBytes(StandardCharsets.UTF_8);
                os.write(input, 0, input.length);
            }

            int responseCode = conn.getResponseCode();
            return responseCode == HttpURLConnection.HTTP_OK;

        } catch (Exception e) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }

    /**
     * 切换 pipeline（无需重新上传地图）。
     */
    public static boolean switchPipeline(int pipelineIndex) {
        HttpURLConnection conn = null;
        try {
            String urlStr = "http://" + LIMELIGHT_IP + ":" + PORT
                    + "/pipeline-switch?index=" + pipelineIndex;
            URL url = new URL(urlStr);
            conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1000);
            return conn.getResponseCode() == HttpURLConnection.HTTP_OK;
        } catch (Exception e) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }
}
```

**关键说明**：

- Limelight 通过 USB 连接 Control Hub 时的默认 IP 为 `172.28.0.1`。如果你使用 Wi-Fi 连接或修改过 IP，需要相应调整。
- `pipeline-switch` 接口同样支持 URL 参数 `index`，切换延迟极低（毫秒级）。
- 上传和切换操作应**在后台线程执行**，避免阻塞 OpMode 的主循环。可以使用 `new Thread(() -> {...}).start()`。

## 注意事项

1. **IP 地址确认**：Limelight 通过 USB 连接 Control Hub 时 IP 为 `172.28.0.1`。若使用 Wi-Fi 连接，需在 Limelight web UI 中查看实际 IP。
2. **线程安全**：`uploadFieldMap` 和 `switchPipeline` 必须在后台线程执行，不能阻塞 OpMode 的 `loop()`。
3. **Pipeline 预加载**：推荐赛前通过 web UI 将两份地图分别上传到 Pipeline 0 和 Pipeline 1。比赛中只需 `pipelineSwitch`，切换延迟在毫秒级，比每次上传地图快得多。
4. **fmap 坐标系统**：Limelight 的 fmap 使用场地中心为原点。BIOBUZZ 官方提供的场地坐标可能需要转换。生成 transform 矩阵时注意单位统一为米。
5. **上传失败兜底**：如果 `switchPipeline` 返回 `false`，应保持当前地图不变，并考虑回退到手动模式，由操作手通过 Driver Station 选择状态。
6. **Tag ID 过滤与地图切换联动**：切换 pipeline 后，`SetFiducialIDFiltersOverride` 的过滤列表也应同步更新，确保只保留当前朝上 CELL 的 4 个 Tag ID。
7. **`LimelightHelpers`** **的可用性**：FTC 环境中 `LimelightHelpers` 并非 SDK 自带，需要将 `LimelightHelpers.java` 文件复制到项目中。如果不想引入该文件，`SetFiducialIDFiltersOverride` 也可以通过 REST API `POST /update-pipeline` 发送 JSON 来实现。

