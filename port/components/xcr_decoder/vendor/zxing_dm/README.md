# DataMatrix finder dependency

Source: [ZXing-C++ v3.1.1](https://github.com/zxing-cpp/zxing-cpp/tree/v3.1.1/core/src),
Apache-2.0 (see LICENSE and each source's copyright notice).

This directory contains the transitive source/header subset needed by the
DataMatrix detector and grid sampler. It does not replace the project's C
ECC200 decoder or its original Gray8 corner refinement.

Local changes:

- DMDetector accepts a cancellation callback, checked on scan lines and candidates.
- PointF and detector/regression/local-grid calculations use single precision
  for the ESP32-P4 hardware FPU. These sources build with
  -fsingle-precision-constant on both P4 and host replay.

The integration in src/xcr_dm_body.cpp supplies its own sparse local-contrast
threshold, accepts only 10x10 symbols, and delegates payload validation to
datamatrix_decode_grid10. The P4 vector threshold has a scalar host equivalent
and is checked against the scalar formula by the board replay application.
