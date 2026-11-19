# DataMatrix candidate binarization and P4 measurements

## Decode path

The ESP32-P4 ISP supplies the 1280 × 720 Gray8 frame. Its gamma mapping is global; it has no adaptive threshold stage. Gray8 is also needed for subpixel corner fitting.

Core 0 produces a detection image at 2 px intervals with a local threshold. A sparse 3 × 3 stencil measures the local gray range, and the P4 vector unit processes 16 pixels per instruction. A ring of sampled rows avoids reading the same Gray8 row three times. Core 1 finds the DataMatrix L border, samples the grid, validates it with the existing ECC200 decoder, and refines the four corners in Gray8. The detector stores each pixel's edge visit state in two bits (57,600 bytes rather than 230,400 for a 640 × 360 detection image). This state is cleared between scan directions and never retained between frames.

When the grid does not pass ECC, the decoder projects the current candidate's 10 × 10 module centers back into the Gray8 image:

- Shortest candidate edge under 45 px: one bilinear sample at each module center to avoid crossing a module edge.
- Edge at least 45 px: average five samples near each module center to reduce noise.
- Estimate lighting variation from the sampled module values. Prefer Otsu for uniform lighting and local min/max for an unevenly lit candidate. Try the other threshold only after an ECC failure. This retry processes 100 module values rather than thresholding the full frame again.

The candidate size estimates the sampling footprint. The candidate's lighting determines the threshold order. Candidate count, sampling offsets and time are bounded. The existing outer marker search is available when detection or corner refinement fails. Each frame starts from its own image and does not reuse the last frame's location, sampling step or threshold.

## Board replay

ESP32-P4 rev 1.3 at 360 MHz, PSRAM at 200 MHz. The test firmware feeds the original 54 grayscale JPEGs from `dist/captures` through hardware JPEG decode, then measures Gray8 input to decoded payload and refined corners. JPEG input time is excluded. The older firmware was measured against the same files; its mean includes failed attempts.

| Firmware | Decoded | Mean | Median | Max |
| --- | ---: | ---: | ---: | ---: |
| Baseline `e77fa4f` | 9/54 | 61.3 ms | 69.7 ms | 95.7 ms |
| Final, forward and reverse | 108/108 | 19.7 ms | 19.1 ms | 27.8 ms |

The final build recovered the 45 failed images as `012348` and retained the nine `012345` results. All 108 results had refined corners. Detected short edges ranged from 36.0 to 196.2 px. All 54 images passed ECC on the first grid, so the candidate threshold retry was not needed for this folder. The replay also checked the P4 vector threshold and row sampler against the scalar formulas.

The 1,185-case host regression retained every baseline success with refined corners (1,033) and produced 1,090 refined results. No false positive was detected on the blank and covered inputs. Separately, earlier board replay of the historical Gray8 corpus after lossy JPEG conversion still had four misses on images containing a label; the 54-image result applies to the specified folder.

## Live camera

OV9281 remains at 1280 × 720, 50 fps capture. Results below count decoded frames received by the host in the current scene. Single frame latency includes the wait between the two cores; the cores also work on different frames concurrently, so throughput is not the reciprocal of latency.

| Image transfer | Decode throughput | Decoded attempts | Mean latency | Core 0 / Core 1 |
| --- | ---: | ---: | ---: | ---: |
| Off | 42.2 fps | 338/338 in 8.016 s | 37.0 ms | 82.2% / 81.9% |
| `@SNAP` every 200 ms | 36.5 fps | 365/365 in 10 s | 42.3 ms | 87.7% / 83.3% |

The second run received 49 pictures in 10 seconds. The 40 fps target was met with upload off. At five snapshot requests per second, picture processing and transfer still reduced throughput below 40 fps.

The normal camera firmware was restored on COM4 after replay. Raw measurements: [baseline](captures_20261119_p4/baseline_replay.log), [final replay](captures_20261119_p4/packed_replay_p4.log), [upload off](captures_20261119_p4/packed_live_off.json), [snapshot upload](captures_20261119_p4/packed_live_on.json), [host regression](captures_20261119_p4/regression_packed.log).
