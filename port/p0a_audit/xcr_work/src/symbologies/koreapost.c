/*
 * Korea Post restoration is not wired yet.
 * Do not fake decode results with sample-signature tables.
 */

#include "koreapost.h"
#include <string.h>

int koreapost_decode(const Image *img, BarcodeResult *result)
{
    (void)img;
    if (result) {
        memset(result, 0, sizeof(*result));
    }
    return 0;
}
