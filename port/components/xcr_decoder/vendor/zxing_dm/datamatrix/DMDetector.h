/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "StdGenerator.h"
#include "DetectorResult.h"
#include <functional>

namespace ZXing {

class BitMatrix;

namespace DataMatrix {

using DetectorResults = std::generator<DetectorResult>;

DetectorResults Detect(const BitMatrix& image, bool tryHarder, bool tryRotate, bool isPure,
                       const std::function<bool()>& cancelled);

} // DataMatrix
} // ZXing
