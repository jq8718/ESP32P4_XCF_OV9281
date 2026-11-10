# ESP32-P4 + OV9281 扫描工程

本工程从 `../ESP32P4_XCF` 的实时扫描固件移植。沿用 XCR 解码器、双核最新帧流水线、按键与状态灯、USB CDC 结果协议和 MJPEG UVC 预览；采集端改为 OV9281 的 1280×720、50 fps、RAW8、双通道 MIPI CSI 模式。默认解码模式与参考工程相同，为 DataMatrix；可在 `XCR_DATAMATRIX_ONLY` 配置中切换至 QR 模式。

## 软件设计

```text
OV9281 RAW8 → ESP32-P4 CSI/MMAP → Gray8 帧 → Core0 采集/预处理
                                             ↓ 最新帧队列
                                     Core1 XCR 解码/定位
                                             ↓
                                  USB CDC 结果 + MJPEG UVC 预览
```

- `port/components/p4scan_camera_capture`：CSI 配置、帧缓冲所有权、缓存同步与曝光接口。尺寸改为 1280×720，OV9281 默认曝光值为 681 行；RAW8 绕过 Bayer ISP，曝光由传感器控制接口管理。
- `port/components/esp_cam_sensor/sensors/ov9281`：传感器寄存器表与自动检测。配置为 24 MHz XCLK、双通道 MIPI、1280×720 RAW8、50 fps。
- `port/components/p4scan_board_power`：新电路无 PCA9538A，不执行扩展器访问；使用 OV9281 的 16 位寄存器地址读取 `0x300a/0x300b` 芯片 ID。
- `port/standalone_idf_test/main`：参考工程的实时解码、反馈及 USB 协议。UVC 为 1280×720 MJPEG、30 fps、HS ISOC；采集 50 fps，预览允许跳帧。
- `port/components/espressif__tinyusb`：保留参考工程对 UVC 视频传输状态和挂起复位的扩展接口，作为本地依赖固定版本。

## 板级接口前提

| 信号 | 当前配置 |
| --- | --- |
| OV9281 MIPI | ESP32-P4 CSI 时钟及两条数据通道 |
| OV9281 XCLK | 模组提供的 24 MHz |
| 摄像头使能 | GPIO0 拉高，等待 100 ms（与 `uvc_example.c` 一致） |
| OV9281 SID | 拉低，7 位 SCCB 地址 `0x60` |
| I2C | GPIO7 SDA、GPIO8 SCL |
| 扫描按键 | GPIO35，低电平有效 |
| 状态灯 | GPIO51 WS2812 |
| USB | 高速设备，CDC + MJPEG UVC |


## 构建与验收

在 PowerShell 执行 `./build_p4.ps1`，产物为 `build/p4scan_ov9281.bin`。工程使用 ESP-IDF 5.4、16 MB Flash，分区表和 CDC 输出格式沿用参考工程。`USB_PROTOCOL.md` 说明主机协议。根目录 `uvc_example.c` 仅作原始示例参考，不参与构建。

上板后依次确认：日志中的 OV9281 ID 为 `9281`；CSI 格式为 1280×720 RAW8 且收到首帧；按键和 WS2812 正常；CDC 扫描结果及 UVC 预览与参考工程协议一致。DataMatrix 已用现场码样验证；QR 模式仍需单独切换配置并上板验证。

新电路无 PCA9538A；瞄准灯和补光灯接口不操作硬件。两条 MIPI 数据通道已由用户确认。

## OV9281 模式依据

1280×720 RAW8、50 fps 的寄存器表及传感器驱动与用户提供的 WT9932P4-TINY_OV9281_UVC 工程逐文件核对一致。参考 UVC 工程的构建记录使用 IDF 6.1；本工程固定使用本机 IDF 5.4，复用 ESP32P4_XCF 后续解码与 CDC 协议。

本机 IDF 5.4 的 CSI DMA burst 配置曾被改为 16；工程内恢复为与 DMA handshake 一致的 512，避免 CSI FIFO 满而没有完整帧。1280×720 RAW8 的每帧有效数据为 921600 字节。

## 当前实机结果

已编译并烧录，确认 CSI 1280×720 RAW8 持续取帧。现场 DataMatrix `012345` 已由设备解码并通过 CDC 输出，UVC 图像及 CDC 角点坐标一致。`tools/check_datamatrix.py 012345` 可复测 UVC 与 CDC 并行链路；详细记录见 `docs/OV9281_VALIDATION.md`，现场图见 `docs/dm_verified_uvc.png`。

Windows UI 调试器为 `dist/P4ScanOV9281Debugger.exe`，源码和使用说明见 `tools/ov9281_debugger/README.md`。它按参考调试器设计，通过 CDC 上传解码处理后的成功/失败同帧图片，手动开关控制上传，不打开 UVC 实时流。UVC 收帧数包含重复预览 JPEG；CDC 只有解码、角点精定位和 100 ms 帧龄检查均通过才发送定位行，因此 219 帧 UVC 与 77 条 CDC 不能直接按差值计算“丢失”的识码结果。
