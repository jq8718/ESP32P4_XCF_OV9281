#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* float(INT32_MAX) rounds UP to 2^31: use an exclusive upper bound.
 * The adjacent float below it is 2147483520, whose rounding is safe.
 * Check before lroundf/cast; NaN, Inf and overflow never wrap or saturate.
 */
static inline bool scan_uart_scaled_i32(float value, float scale, int32_t *output)
{
    float scaled = value * scale;
    if (!isfinite(scaled) || scaled < -2147483648.0f || scaled >= 2147483648.0f)
        return false;
    *output = (int32_t)lroundf(scaled);
    return true;
}
