# AGV 左右并行标记搜索（2026-11-10）

## 实现

AGV 实时模式仍由 Core 0 经 CSI/DMA 取 1280×720 Gray8 帧，Core 1 持有最新帧并启动本帧解码。定位标记检测把约 214×120 的六像素采样网格分为左右两块：Core 1 搜索左块，Core 0 的常驻任务搜索右块。两块在图像中线处各延伸 40 个采样列，即 240 个原图像素；每块使用独立的掩码和连通域队列，不同时改写同一工作区。等待两侧完成后按候选几何评分选用较佳码区，Core 1 继续归一化、ECC200 解码和原图角点精修。左右均未找到码区，或分区候选未能通过最终码字解码时，回退原整图搜索。没有使用上一帧位置的局部 ROI。

实现见 `port/components/xcr_decoder/src/xcr_dmtx.c` 的 `find_marker_roi_slice()` 和 `find_marker_roi_parallel()`。右块任务优先级低于取帧和 CDC；解码任务持有相机帧直到右块任务完成，避免 DMA 缓冲被提前归还。非 AGV 模式仍走原整图搜索。若创建右块任务失败，也自动使用整图搜索。

## 核对

主机测试用一张真实 1280×720 Gray8 帧，水平平移 0～600 像素，覆盖码在左侧、跨中线和右侧的 11 个位置；串行整图与左右分区的载荷、ROI 和四角完全相同。空白帧及遮挡帧均未解码。另将图中的标记区域放大三倍并置于中线，分区搜索未命中时回退整图搜索，结果与整图路径一致。程序为 `tools/check_marker_split.py`，结果在 [marker_split_host_results.json](decode_stages/marker_split_host_results.json)。这些变换样本核对了代码分区边界与回退路径；实际 AGV 移动时的模糊、曝光变化仍需现场验证。

IDF 5.4 固件编译并烧录到实机后，COM57 连续 20 秒有 20 条阶段抽样，均找到标记、完成角点精修、解出 `012345`；无整图回退、无重启。此时码位于画面左侧。阶段中位数：并行标记搜索 11.284 ms（Core 0 右块 11.219 ms，Core 1 左块 10.586 ms），归一化 2.763 ms，码字解码 2.079 ms，角点精修 2.506 ms，处理合计 18.754 ms。此前串行抽样的标记搜索中位数为 13.595 ms、处理合计 19.806 ms；画面条件可能有差异，不能把全部差值归因于分区。

补齐解码失败回退后重新构建并刷入最终固件，Flash hash 校验通过。随后 12 秒得到 11 条阶段抽样，全部并行搜索成功、精修成功、解出 `012345`；无整图回退或重启。最终版中位数：标记搜索 11.110 ms，Core 0 右块 11.047 ms，Core 1 左块 10.653 ms，归一化 2.778 ms，码字解码 2.056 ms，角点精修 1.290 ms，处理合计 17.377 ms。相比此前串行抽样，标记阶段约缩短 2.5 ms；不同采样窗口不构成严格同场景 A/B 测试。

最终固件与 Windows 调试器联机 8 秒，收到 322 条成功定位、21 张同帧成功图片，双核占用率正常更新；界面截图见 [parallel_marker_ui_smoke/window.png](parallel_marker_ui_smoke/window.png)。

HS CDC 的开图/关图交替测试每段预热 2 秒、采样 6 秒，见 [parallel_marker_upload_benchmark.json](parallel_marker_upload_benchmark.json)。关图两段成功结果均为 283 条（47.17 次/秒），解码中位数分别为 18.64 和 19.26 ms；Core 0 约 60%，Core 1 约 95%。先前串行版本在同类测试中关图为 272 条（45.33 次/秒）、约 20.44 ms，Core 0 约 13.3%，Core 1 约 100%。开图时结果频率受 JPEG/CDC 和现场遮挡影响，不能据此判断算法本身收益。

## 使用边界

两块仍分别计算自身的采样、局部阈值和连通域，中线重叠会重复工作；因此 Core 0 占用明显上升，实际单帧提升约 1～2 ms，并非按核数减半。目标大到两侧都无法覆盖三个标记时会回退整图，正确性优先但该帧耗时更长。当前板端场景和主机变换样本已通过；不同码尺寸、强运动模糊、多码和逆光仍需在 AGV 行驶条件下核对。

## 复现主机边界测试

在工程根目录运行：

```powershell
wsl -e bash tools/build_dm_stage_trace.sh
wsl -e env XCR_DM_SPLIT_TEST=1 bash tools/build_dm_stage_trace.sh
python tools/check_marker_split.py
```

脚本使用仓库中的 `docs/decode_stages/source.gray8` 生成平移、遮挡和扩大样本；中间图放在被 Git 忽略的 `build/parallel_host_cases`。
