# STM32 接收 P4 UART v6

将 `p4_scan_protocol.c/.h` 加入 STM32 工程，不依赖 HAL、RTOS 或动态分配。UART 设置为 921600、8N1；DMA + IDLE / 环形缓冲积累数据后，在主循环或通信任务中按 LF 拆出完整行，调用 `p4_scan_parse_line()`。函数返回 1 才能使用输出，返回 0 为其他消息，返回 -1 丢弃损坏行。解析函数不放在 ISR 内运行。

新固件默认 COMPACT，直接收帧号、内容、偏航毫度、码平面垂直距离毫米和沿过道毫米。需要全量时通过 TX→P4 GPIO42 发送 `@OUT,FULL\r\n`；恢复精简发送 `@OUT,COMPACT\r\n`。

`p4_scan_parse_line()` 自动识别 `C`（Compact，精简）与 `F`（Full，完整）。`compact == 1` 时只有 `yaw_mdeg/plane_mm/along_mm` 是已上传的姿态字段；`compact == 0` 时还包含四角点和其他姿态字段。角点仍按 P0→P3 身份排序。`pose_valid == 0` 时不使用姿态数据。

`payload` 是原样传输的 ASCII 数字，例如 `012345`。长度由 `payload_length` 给出，不保证以 NUL 结尾。显示时可用 `printf("%.*s", (int)result.payload_length, (const char *)result.payload)`，按字符串处理可保留前导零。

码体有效边长根据内容最左边一位选择，不含白边和外部定位标记：

| 首字符 | 例子 | 边长 |
|---|---|---|
| `0` | `012345` | 30 mm（3 cm） |
| `1` | `112345` | 50 mm（5 cm） |
| `2` | `212345` | 70 mm（7 cm） |

解析后可调用 `uint16_t side_mm = p4_scan_code_side_mm(result.payload, result.payload_length);` 获取尺寸；空内容或其他首位返回0。该函数适用于C和F两种模式。姿态有效性仍由 `result.pose_valid` 判断；尺寸由P4用于距离解算，STM32直接使用收到的毫米值，不应再乘尺寸比例。

每行允许至多 1024 字节（含 CRLF），接收缓冲请至少为 1024 字节，溢出时丢弃至下一行再恢复。帧号范围为 0～999，`999 → 0` 正常回绕。上层可用 `(current + 1000 - previous) % 1000` 算短时间内的间隔，通信超时按接收时间判断。CRC 校验通过不代表二维码定位精度达标。

角度单位是 0.001°，距离单位是 mm，角点单位是 0.1 px。控制可始终用 int32；只有显示或现有浮点控制接口需要时才除以 1000.0f。详细布局、CRC 范围与内参命令见 [STM32_UART_PROTOCOL.md](../../docs/STM32_UART_PROTOCOL.md)。
