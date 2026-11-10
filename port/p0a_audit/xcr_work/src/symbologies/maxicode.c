/*
 * MaxiCode restoration is not wired yet.
 * Do not fake decode results with sample-signature tables.
 *
 * Note: the original library cannot decode either sample in the coverage
 * test set (maxicode.bmp / maxicode_inv.bmp are both MISS against
 * xcf-original), so MaxiCode is out of scope for restoration parity.
 */

#include "maxicode.h"
#include <string.h>

int maxicode_decode(const Image *img, BarcodeResult *result)
{
    (void)img;
    if (result) {
        memset(result, 0, sizeof(*result));
    }
    return 0;
}
