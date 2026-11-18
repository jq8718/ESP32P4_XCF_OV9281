# ESP32-P4 OV9281 扫码头 CDC 对接协议

版本：1.0，2026-11-11。适用于当前 ESP32P4_XCF_OV9281 的连续扫码固件，当前识别 DataMatrix。本文按现有固件说明，不增加新命令或修改固件。

## 1. 接口与连接

摄像头扫码头为 **USB Device**，STM32 为 **USB Host**。使用扫码头的 HS USB 数据口连接 STM32 USB Host；FS USB Serial/JTAG 口用于烧录和日志，不用于本协议。

这不是 TTL UART，不能把 USB D+/D− 接到 STM32 USART TX/RX。STM32 工程需要 USB Host CDC ACM 驱动，而不是 USB Device CDC 驱动。

| 项目 | 当前值 |
| --- | --- |
| VID / PID | `0x303A / 0x8001` |
| 产品名称 | `P4SCAN OV9281 UVC+CDC` |
| USB 设备类 | 复合设备，Device Class `0xEF`，Subclass `0x02`，Protocol `0x01` |
| CDC 控制接口 | 当前 interface 2；Class `0x02`、Subclass `0x02`、Protocol `0x00` |
| CDC 数据接口 | 当前 interface 3；Class `0x0A` |
| 通知端点 | `0x83`，Interrupt IN |
| STM32 → 扫码头 | `0x04`，Bulk OUT |
| 扫码头 → STM32 | `0x84`，Bulk IN |
| Bulk 最大包长 | HS：512 字节；FS：64 字节，以枚举描述符为准 |
| Line Coding 建议 | 921600、8N1；为 CDC 设置，不限制 USB 实际吞吐 |
| DTR / RTS | 当前固件不依赖 DTR；建议 Host 按常规置 DTR=1、RTS=0 |
| 设备序列号 | MAC 的 12 位十六进制文本加 `-OV9281`；当前设备 `30EDA0E3455F-OV9281` |

STM32 应遍历配置描述符，选择 CDC 接口及其端点。UVC 位于前面的 interface 0/1，不能只检查设备类是否为 CDC，也不能假设 CDC 一定是 interface 0。使用 STM32Cube USB Host 中间件时，需要核对该版本是否支持从此 UVC+CDC 复合设备中选择 CDC 功能。当前 CDC 接口 Protocol 为 `0x00`，若驱动硬编码匹配 `0x01`（AT Command），需要适配接口选择条件；不要把设备级 Protocol `0x01` 和 CDC 接口级 Protocol 混淆。

STM32 Host 负责正确管理 USB VBUS 与供电。只接收结果时不打开 UVC 视频流。FS 的文本结果带宽足够，固件已按 FS 返回 64 字节 Bulk 描述符；具体 STM32 型号、USB 控制器及 Host 中间件尚未实板验证。

## 2. 推荐上电流程

1. USB 枚举完成，打开 CDC Bulk IN/OUT，设置 Line Coding。
2. 持续接收 Bulk IN，按 **`0x0D` 回车**重组完整行。
3. 依次发送以下命令；每条实际以一个 `0x0D` 结束：

```c
/* 每条发送完成后再提交下一条；字符串结束 NUL 不发送。 */
"@DEBUGOFF\r"      /* 关闭调试图片请求 */
"@PROFILEOFF\r"    /* 关闭分段诊断 */
"@TIMINGON\r"      /* 开启耗时与扫描状态 */
"@GEOMETRYON\r"    /* 开启 DataMatrix 定位质量，结果变为 17 字段 */
"@SUBPIXELON\r"    /* 坐标使用小数，最多小数点后 3 位 */
```

4. 收到 17 字段结果，并通过第 5 节检查后，再发布给 AGV 控制任务。

命令无 ACK、无回显、无事务编号；未知命令被忽略。以实际收到的字段格式判断配置是否生效。命令大小写敏感，不加空格，接收端也接受 LF 作为命令结束符。单条命令正文最多 31 字节。

扫码头上电自动连续扫码，不需要 START 命令。上述命令只控制输出格式和诊断，不控制解码启停。每次 USB 重新枚举后都重新初始化；接收端兼容初始化过程中的 12/13 字段，等待收到 17 字段再进入正式定位状态。

## 3. 基本分帧规则

- 文本行以 `CR=0x0D` 结束，设备不附加 LF 或 NUL。正文为 CSV，数字为十进制 ASCII，字段之间为英文逗号 `0x2C`。
- **USB 包不是协议帧**：一行可能跨多次接收，一次接收也可能包含多行。
- 原始载荷最多 255 字节；转义后可能超过 255 字节。建议行缓冲 1536 或 2048 字节，载荷缓冲 256 字节并另存实际长度。
- 行溢出时丢弃到下一个 CR 后再恢复，不能将截断行用于定位。
- 普通结果行没有应用层 CRC、长度头、重传或 ACK。USB 自身处理链路校验，应用仍须验证字段个数、数值范围和时间有效性。

## 4. 扫码头 → STM32：定位结果

### 4.1 默认格式：12 字段

```text
payload,x0,y0,x1,y1,x2,y2,x3,y3,sequence,capture_us,age_us<CR>
```

`<CR>` 只表示一个回车字节，不是实际发送的四个字符。以下同。

### 4.2 开启耗时：13 字段

发送 `@TIMINGON`，保持 `@GEOMETRYOFF` 时：

```text
payload,x0,y0,x1,y1,x2,y2,x3,y3,sequence,capture_us,age_us,decode_us<CR>
```

### 4.3 推荐定位格式：17 字段

发送 `@GEOMETRYON` 后，当前 DataMatrix 结果输出：

```text
payload,x0,y0,x1,y1,x2,y2,x3,y3,sequence,capture_us,age_us,decode_us,quality,edge_mask,support,residual_milli<CR>
```

即使关闭 `@TIMINGON`，只要 `@GEOMETRYON` 仍有效，17 字段中也保留 `decode_us`。坐标是否带小数由 `@SUBPIXELON/OFF` 独立控制。

| 字段序号（从 0 开始） | 名称 | STM32 存储建议 | 含义 |
| --- | --- | --- | --- |
| 0 | payload | `uint8_t[256]` + 长度 | 解码内容，先 CSV 解析再还原转义 |
| 1～8 | x0,y0…x3,y3 | `int32_t`，单位 0.001 像素 | 原图四个角点；整数模式接收后乘 1000 |
| 9 | sequence | `uint32_t` | 采集帧序号；重启后重置，允许跳号与自然回绕 |
| 10 | capture_us | `uint64_t` | 设备启动后的帧完成时间，单位 µs |
| 11 | age_us | `uint64_t` | USB 写入前格式化时的设备时间减 capture_us，单位 µs |
| 12 | decode_us | `uint32_t` | 本帧前段搜索至后段精修完成的时间，含两核交接等待，单位 µs |
| 13 | quality | `uint8_t` | 0 无定位；1 粗定位；**2 四边精修通过** |
| 14 | edge_mask | `uint8_t` | bit0～bit3 为 P0→P1、P1→P2、P2→P3、P3→P0，15 表示四边齐全 |
| 15 | support | `uint16_t` | 四边拟合支持点总数；不是百分比或成功率 |
| 16 | residual_milli | `uint32_t` | 各边拟合残差均值中的最大值，放大 1000；683 表示约 0.683 像素 |

数值示例（17 字段）：

```text
012345,1074.125,567.250,744.000,608.500,688.750,336.125,959.500,223.000,1032,20972437,27500,26973,2,15,100,683<CR>
```

该例载荷为 `012345`，P0 为 `(1074.125,567.250)`，帧号 1032，帧完成时间 20.972437 秒，输出时年龄 27.5 ms，本帧定位耗时 26.973 ms。示例数值用于协议说明，不构成某张现场图的测量记录。

### 4.4 坐标与角点顺序

图像为 **1280×720**。左上原点，X 向右、Y 向下，`0≤x<1280`、`0≤y<720`。坐标已映射回原图，不需要 STM32 再叠加 ROI 或降采样偏移。

P0、P1、P2、P3 沿码体边界依次排列，随标签旋转，不按图像左上角重新排序。当前 DataMatrix 输出中，P3 对应两条实线边的交点，P1 对应两条时钟边的交点；P0/P2 为另两个角。P0→P1、P1→P2 为时钟边，P2→P3、P3→P0 为实线边。四角是码体模块区域外边界，不包含白色静区和三个外部标记。

协议传送的是像素位置；毫米距离、偏航角和三维姿态需要相机内参、畸变及标签物理尺寸标定。不要仅依据屏幕上四角的上下左右关系计算标签方向。

## 5. STM32 有效性与超时处理

默认 12/13 字段模式下，当前 DataMatrix 固件只发送精修通过的定位结果。**开启 `@GEOMETRYON` 后，允许输出粗定位结果用于诊断，STM32 必须主动筛选 `quality==2`，不能只看收到 CSV 就认定定位有效。**

推荐接收条件：

1. CSV 正确，字段数为 17，载荷长度 1～255 字节，坐标均在原图范围内。
2. `quality==2`、`edge_mask==15`；当前协议解析器还检查 `28≤support≤128`、`0≤residual_milli≤1500`。
3. `age_us<100000`，这是当前固件帧龄上限；AGV 可以按控制需求采用更严格门槛。
4. 帧序号不重复；在同一连接会话内结合 capture_us 判断先后顺序。跳号可能来自无结果、超龄或最新结果覆盖，不能单独视为通信错误。
5. 保存 STM32 本地接收时间。消费结果时，设备给出的 age_us 加本地接收后的等待时间，也不能超过应用门槛。该估计**仍不含 USB 传输与 Host 排队延迟**；严格跨设备时间配准需要另行时钟同步。

设备队列保留最新结果，忙时覆盖，不缓存等待重传。无内容、超时或精修失败时默认不发定位行。不能认为最后一次有效位置会永久有效；例如可将“连续 100 ms 未收到新有效定位”作为失效条件，具体时间由 AGV 控制要求确定。

USB 断开立即清除定位有效标志和接收半行。设备时间或帧号重置时建立新会话，不能把重启前后的时间直接相减。

## 6. 载荷转义：必须先解析 CSV

载荷含逗号或双引号时，第一字段用双引号包裹，内部双引号写成两个双引号。不能用 `strtok(line, ",")` 代替 CSV 解析。

控制字节 `0x00～0x1F`、`0x7F` 写成四个 ASCII 字符 `\xHH`，反斜杠写为 `\\`。接收端先去 CSV 引号、还原双引号，再还原反斜杠转义。载荷不是必须为 UTF-8；按原始字节保存，嵌入 NUL 时仍按长度处理。

| 原始载荷 | 第一字段在线路上的文本 |
| --- | --- |
| `012345` | `012345` |
| `A,B` | `"A,B"` |
| `A"B` | `"A""B"` |
| 字节 `41 0D 42` | `A\x0DB` |
| 字节 `41 5C 42` | `A\\B` |

## 7. 其他主动上报

### 扫描状态（启用 @TIMINGON）

```text
@TIMING,sequence,capture_us,decode_us,success<CR>
```

`success` 为 0/1，当前实现按本帧是否仍保留解码结果设置；超龄结果会先清除。**1 不保证四边精修通过**，也不保证对应定位 CSV 已发出；必须以定位行的 quality 判断能否用于 AGV。该消息队列只保留最新一条，拥堵时可丢弃，不能按消息数量计算全部相机帧成功率，也不替代定位 CSV。

### 双核占用（约每秒自动上报）

```text
@CPU,interval_ms,core0_permille,core1_permille<CR>
@CPU,1000,905,545<CR>
```

上例为 Core 0 90.5%、Core 1 54.5%。单位为千分比，范围 0～1000；不需要发送开启命令，无独立关闭命令，接收端可忽略。此消息可作通信活动参考，但不能证明当前看到了码或定位有效。

### 分段诊断（启用 @PROFILEON）

```text
@PROFILE,sequence,frontend_us,wait_us,normalize_us,core_us,refine_us,sample_step,attempts<CR>
@FRONT,sequence,sample_us,binary_us,components_us,geometry_us<CR>
```

约每秒最多一条，分别表示前段搜索、两核交接等待、归一化准备、内容解码、四角精修、搜索间隔和归一化尝试次数。正常 STM32 控制流程保持关闭。

分流时先尝试验证 12/13/17 字段定位 CSV，再按完整消息名及字段数量识别诊断消息。合法载荷可以以 `@` 开头，不能直接丢弃所有 `@` 开头的行。

## 8. STM32 → 扫码头：命令表

下表正文后均发送 CR，不发送字符串结尾 NUL；均无 ACK。

| 命令正文 | 效果 |
| --- | --- |
| `@DEBUGOFF` | 关闭后续 JPEG 请求并取消待处理 RAW 请求；在途数据仍可能发送完 |
| `@TIMINGON` / `@TIMINGOFF` | 开关耗时字段与 @TIMING 消息 |
| `@GEOMETRYON` / `@GEOMETRYOFF` | 开关 DataMatrix 的四个定位质量字段 |
| `@SUBPIXELON` / `@SUBPIXELOFF` | 开关精修角点的三位小数输出；关闭后输出整数像素 |
| `@PROFILEON` / `@PROFILEOFF` | 开关分段计时诊断 |
| `@SNAP` | 请求一张成功/失败解码同帧 JPEG，不启动实时视频 |
| `@RAW` | 请求一张后续处理帧的完整 Gray8 原图 |
| `@RAWFAIL` | 请求一张后续无解码结果或因超龄被清除结果帧的 Gray8 原图；精修失败但仍有解码结果时用 @RAWCOARSE |
| `@RAWCOARSE` | 请求一张后续内容已解码但角点精修未通过帧的 Gray8 原图 |

当前没有设置曝光、补光、分辨率、帧率、START/STOP、PING、版本查询或应用复位命令。未实现的指令不能据此假定已生效。USB 断开会关闭耗时、定位质量和分段诊断输出；子像素状态不应依赖断线复位，每次连接显式设置。

## 9. 图片扩展（STM32 仅做定位时可忽略）

发送 `@SNAP` 后，通常返回：

```text
@FRAME,1,sequence,capture_us,width,height,success,decode_us,x0,y0,x1,y1,x2,y2,x3,y3,payload_hex,jpeg_length,crc32_hex<CR>
@DATA,base64_chunk<CR>
...多条 DATA...
@END,sequence<CR>
```

启用定位质量且该帧为 DataMatrix 时，头部版本为 2，并在 crc32_hex 后追加 `quality,edge_mask,support,residual_milli`，共 23 字段（含 @FRAME）。版本 1 共 19 字段。

每条 DATA 独立 Base64 编码最多 384 个 JPEG 原始字节；按顺序解码拼接。验证总长度、结束帧号及 IEEE CRC32（与 Python zlib.crc32 相同）后才使用。普通定位 CSV 可以穿插在 DATA 行之间，仍需及时处理。设备单次 JPEG 发送预算约 2 秒；不完整图片丢弃。资源忙或暂无帧时可能返回 `@WAIT,busy`、`@WAIT,no_frame`。

JPEG 为 1280×720；success=0 时不能用作有效定位。当前实现的 success=1 表示元数据中有解码内容和几何信息，**可能仍是粗定位**；用于定位时必须检查版本 2 的 quality==2，或关联同帧的有效定位 CSV。`@SNAP` 请求租期约 3 秒，每次请求只消费一次编码请求，不触发扫码或点灯；连续请求可合并，不保证一命令对应一张独立帧。

RAW 格式为：

```text
@RAWFRAME,sequence,capture_us,1280,720,success,decode_us,921600,crc32_hex<CR>
@RAWDATA,base64_chunk<CR>
...多条 RAWDATA...
@RAWEND,sequence<CR>
```

RAW 为逐行 Gray8，像素从左至右、从上至下，共 921600 字节，无行填充；每段同样最多 384 原始字节独立 Base64 编码。设备 RAW 发送预算约 5 秒；等待合适失败帧可能更久，也可能一直没有对应场景。STM32 无需图片时不要发送这些命令。

## 10. 接收流程参考

USB 接收回调只搬运到环形缓冲并按 Host 中间件要求续收；在任务中组行和解析，不在中断里做 CSV、浮点或 AGV 控制。

```text
USB 枚举完成
    → 配置 CDC，启动持续 Bulk IN，逐条提交初始化命令
每次接收若干字节
    → 写入环形缓冲（不能假设以 NUL 结束）
    → 遍历字节，以 CR 组行，超长行丢弃至下一个 CR
    → CSV 解析，识别 12 / 13 / 17 字段结果
    → 若是 17 字段：检查 quality、mask、坐标、age 与帧序号
    → 全部通过：原子更新“最新定位 + 本地接收时间 + 有效标志”
    → 否则按完整名称处理 @CPU / @TIMING 等消息
控制任务取用前
    → 再检查新鲜度与 USB 连接状态
```

建议使用毫像素定点坐标，例如 `1074.125` 解析为 `1074125`。时间字段使用 64 位解析，不能用 32 位 atoi 接收 capture_us；32 位微秒值约 71.6 分钟即会溢出。接收、解析和控制任务之间发布结果时应防止读到更新一半的结构体。

本协议及示例已按当前 `scan_usb.c`、USB 描述符和 PC 调试器解析器核对。具体 STM32 USB Host 驱动的初始化代码取决于 MCU 型号和所用 STM32Cube/RTOS 版本。
