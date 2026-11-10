# OV9281 实机验证（2026-11-10）

- 工程：ESP32P4_XCF_OV9281；ESP-IDF 5.4.0（本机 SDK 的 git 版本串为 v1.0.0-dirty，版本依据 esp_idf_version.h 的 5/4/0）。
- 参考：用户提供 WT9932P4-TINY_OV9281_UVC；其构建记录为 IDF 6.1。
- OV9281：两条 MIPI 数据通道、24 MHz 外部时钟、1280×720 RAW8、50 fps；该模式的寄存器表和 ov9281.c 与参考一致。
- 电源：GPIO0 高电平，延时 100 ms；新板无 PCA9538A，不访问扩展器。
- 已完整烧录 IDF 5.4 bootloader、分区表及应用，后续仅更新应用。
- CSI 首帧为 921600 字节，stride 1280；连续帧间隔约 19.9 ms。
- UVC：1280×720 MJPEG、30 fps 描述符、HS ISOC，与 CDC 同时枚举。
- 使用 VID/PID 303A:8001 枚举解析 DirectShow 索引，确认不是 ToDesk 虚拟摄像头。
- 无码阶段的初步检查读取了 90 帧；该测试见 uvc_check.json，后续有真实码样的检查见下文。
- FFmpeg DirectShow 复制录制在本机曾输出全零载荷，不能以 AVI 文件生成作为成像通过依据；最终依据明确设备身份后的 OpenCV 图像解码及人工查看。

## 已修复

CSI bridge burst 恢复为 IDF 5.4 的 512，以匹配 DMA handshake，解决 FIFO 满而无完整帧。
IDF 5.4 的 Gray8 JPEG 预览出现横坐标压缩，已恢复 Gray8 扩展到 RGB888 后编码；现场 UVC 坐标与 CSI/CDC 坐标一致。UVC 发送优先级 9，TinyUSB 优先级 10。
CDC 结果任务优先级升至 9，解决采集期间低优先级任务无法及时发送结果的问题。
DataMatrix 快速定位的采样步长从 12 改为 6，适应本次约 20 像素宽的外部定位标记；保留每四次的标准定位回退。
独立采集预览与解码快照保留不同调用入口；CDC 快照租期内使用解码帧生成快照，避免丢失坐标和耗时含义。
1280×720 的采集、解码输入、快照和 USB 帧尺寸已同步，JPEG 容量为 1843200 字节。

## 重跑

在 PowerShell 执行 `./build_p4.ps1`；运行 `tools/check_uvc.py` 可重启板子，记录 CSI 日志、保存 UVC 图像并测量两种帧率。现场 DataMatrix 检查运行 `tools/check_datamatrix.py 012345`，输出 `dm_check.json`、`dm_verified_uvc.png` 和 `dm_verified_cdc.log`。

## 现场 DataMatrix 验收

- 2026-11-10，当前镜头前码样经独立 ZXing 解码为 `012345`。
- 快速定位调整后，8 秒串口日志显示 155 次 `status=OK`、155 次 `dm_marker found=1`，CDC 实收 143 条 `012345`。角点精定位质量为 `2`，四边掩码为 `15`。
- UVC 与 CDC 并行时，再次读到 218 帧 UVC 图像及 75 条 CDC 结果；UVC 图像中的码样也解为 `012345`。
- 最终固件的自动检查 `dm_check.json`：8 秒收到 219 帧 UVC，抽样解出 21 帧，CDC 收到 77 条；UVC 码范围 `[558,339,662,442]`，CDC 范围 `[558,338,661,441]`，帧龄低于 100 ms。
- USB 协议与 DataMatrix 成功输出已实机确认。QR 模式、按键及无扩展器的新板灯光行为尚未逐项验证。
- 最终烧录应用 SHA256：`064cbc5ca3cd58cbb0172768b772cdba81817312834c2e0a00e4e0c93b3eba64`。
