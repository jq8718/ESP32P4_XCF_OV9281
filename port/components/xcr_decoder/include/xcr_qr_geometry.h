#pragma once
#include <stdint.h>
#include "xcr_port.h"

int xcr_qr_refine_corners(const uint8_t *pixels, int width, int height, int stride,
                          int corners[8], const XcrDeadline *deadline);
