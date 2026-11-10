# AGV 两级跨帧流水线实测（2026-11-10）

后续暗光小码场景与性能修复见 [速度诊断](failure_diagnosis/SPEED_ANALYSIS.md)。本文表格是此前固定场景的历史实测；当前 JPEG 任务优先级已由 9 降为 7，低于解码任务的 8。

当前默认固件在图片上传开启和关闭时，均达到约 50 次成功定位/秒。测试使用现场持续放置的 DataMatrix `012345`、1280×720 Gray8、标称 50 fps，ESP-IDF 5.4 编译并刷入 COM57，HS CDC 为 COM60。结果针对该场景；AGV 行驶时的运动模糊、逆光和距离变化仍需现场验证。

## 工作分配

- Core 0：CSI/MMAP 取帧、整帧外部标记搜索，以及配置在该核的 TinyUSB 端点任务。
- Core 1：归一化和 Otsu 二值化、ECC200 解码、原图角点精修、反馈；JPEG 编码和 CDC 图片/定位发送任务也在该核运行。
- 每帧的标记 ROI、方向和坐标轴保存在对应的相机缓冲槽；后段校验原图指针、序号和尺寸，使用该帧结果继续处理，不重复搜索，也不使用上一帧 ROI。
- 相机由三缓冲改为四缓冲，多用 921600 字节 PSRAM。给前处理、后处理/图片复制和 DMA 留出同时持帧的空间；仍只保留最新的待处理帧，不建立长队列。图片从原图复制到独立预览缓存后，相机缓冲才可归还，避免 DMA 改写正在上传的图。

Core 1 完成帧 N 的后段时，Core 0 可以处理帧 N+1。现场测得两段总延迟约 23～24 ms，而取帧周期约 19.9 ms，跨帧重叠使结果间隔维持约 19.9 ms。两核工作量没有刻意分成各 50%；开图时 Core 0 约 90%，Core 1 约 53%。

相关实现位于 `port/standalone_idf_test/main/agv_pipeline.c`、`latest_frame.h`、`scan_usb.c` 与 `port/components/xcr_decoder/src/xcr_dmtx.c`、`xcr_decoder.c`。并发分配的内存统计已加锁。

## 图片上传处理

只把搜索移到 Core 0 尚不足以在开图时达到 50 次/秒。最终版本还做了以下配合：

1. 默认关闭每 50 帧的同步详细串口日志。初始试验中这一批打印会占住解码线程约 55 ms。`@CPU` 和 `@TIMING` 的 CDC 数据继续输出。
2. JPEG/RAW 的 CRC32 改用 `esp_rom_crc32_le(0, ...)`，替代每字节八轮的逐位计算；格式仍是 IEEE CRC32。最终已验证 JPEG 的尺寸/CRC，也实测取出 921600 字节 RAW 并通过主机 zlib CRC 校验。
3. CDC 图片打包/发送放到 Core 1，优先级 7，低于解码任务 8 和 JPEG 任务 9。TinyUSB 端点任务仍保持原配置的高优先级。
4. 增加第四块相机缓冲。三缓冲试验在开图时约 47.5 次/秒；其他条件不变改为四缓冲后达到约 50.3 次/秒。

“开图”沿用调试器每约 0.35 秒请求一次成功/失败同帧图片，实际本轮约 2.7 张/秒；没有开启实时 UVC 流，也没有把 50 次/秒定位等同于每秒上传 50 张图片。

## 最终交替测试

每段预热 2 秒，再采样 20 秒，按开图→关图→开图→关图测试。原始结果为 [frame_pipeline_final_benchmark.json](frame_pipeline_final_benchmark.json)。

| 段 | 图片上传 | 成功结果数 | 成功结果/秒 | 图片数 | Core 0 | Core 1 | 解码中位数 | 帧龄中位数 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 开 | 1006 | 50.30 | 54 | 90.0% | 53.0% | 23.73 ms | 25.36 ms |
| 2 | 关 | 1007 | 50.35 | 0 | 82.8% | 42.5% | 22.89 ms | 23.61 ms |
| 3 | 开 | 1006 | 50.30 | 54 | 90.1% | 53.2% | 23.83 ms | 26.11 ms |
| 4 | 关 | 1007 | 50.35 | 0 | 82.8% | 42.5% | 22.86 ms | 23.59 ms |

四段合计 4026 条成功定位，108 张成功图片；图片 CRC/尺寸/分包错误为 0，定位序号无重复、无应用取帧序号缺口。开图定位最大帧龄为 45.86 ms，关图为 25.06 ms，低于 100 ms 有效期。设备时间统计的应用取帧频率约 50.323 次/秒，所以短窗口计数略高于标称 50 fps。此应用序号只在取出帧时增加，不能据此推断 CSI 硬件从未丢帧。

开图时独立 `@TIMING` 消息少于成功定位条数，因为图片发送期间它仍采用长度 1 的最新值队列，允许覆盖；定位结果在图片分包间及时发送，最终序号检查没有缺口。结果频率使用定位 CSV 计数，不用 `@TIMING` 或图片数量推算。

## 与先前版本比较

本次动手前的左右并行版本同场景测得：开图 43.12～43.25 次/秒，关图 48.12～48.25 次/秒，见 [frame_pipeline_baseline_benchmark.json](frame_pipeline_baseline_benchmark.json)。只去掉同步诊断和加速 CRC、仍保持左右并行搜索时，关图已经达到约 50.25 次/秒，但开图约 45.12～45.50 次/秒，见 [frame_pipeline_optimized_benchmark.json](frame_pipeline_optimized_benchmark.json)。

因此，达到当前结果是任务分配、图片发送和缓冲管理共同作用，不能全部归因于跨帧流水线。原左右并行版单帧解码约 18～19 ms，新版约 23～24 ms；新版的目标是使上传图片时仍能连续输出约 50 次成功定位/秒，单帧算法延迟没有降低。

## 构建、核对和回退

`P4SCAN_FRAME_STAGE_PIPELINE=y` 默认启用当前两级流水线；关闭可回到左右并行标记搜索。`P4SCAN_PIPELINE_DIAGNOSTICS` 默认关闭，可打开查看各阶段详细 FS 日志，但这些同步打印会影响性能测试。修改 Kconfig 后需 `idf.py reconfigure` 再构建。当前产物为 `build/p4scan_ov9281.bin`，实验前的板上应用备份在 `build/baseline_before_frame_pipeline.bin`。

主机用同一真实图的 11 个横向位置、空白、遮挡和放大跨中线样本，对照整帧搜索与预处理后解码，共 14 个样本的内容、ROI 与角点完全一致，结果为 [marker_prepared_host_results.json](decode_stages/marker_prepared_host_results.json)。可通过以下命令复现：

```powershell
wsl -e bash tools/build_dm_stage_trace.sh
wsl -e env XCR_DM_SPLIT_TEST=1 bash tools/build_dm_stage_trace.sh
python tools/check_marker_split.py
python tools/ov9281_debugger/bench_upload.py --port COM60 --seconds 20 --output docs/frame_pipeline_final_benchmark.json
```

最终也使用现有 Windows EXE 联机 12 秒，确认收到图像和双核占用率。此次界面记录中码的位置已变化，收到 529 条定位、25 张图片（22 张成功、3 张未解码），没有用它替代前面固定场景的性能对照。截图为 [frame_pipeline_ui_smoke/window.png](frame_pipeline_ui_smoke/window.png)，原始计数见同目录 `smoke.json`。这也表明约 50 次/秒是固定可解码场景的吞吐量，不能承诺移动/遮挡时每帧都成功定位。
