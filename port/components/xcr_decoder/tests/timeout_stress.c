#include "xcr_decoder.h"
#include "xcr_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    int iterations = argc > 1 ? atoi(argv[1]) : 10000;
    uint8_t pixels[97 * 32];
    XcrGrayImage image = {0};
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    XcrMemoryStats before;
    XcrMemoryStats after;
    XcrMemoryStats after_destroy;
    int index;

    memset(pixels, 127, sizeof(pixels));
    image.width = 96;
    image.height = 32;
    image.stride = 97;
    image.pixels = pixels;
    xcr_decoder_config_default(&config);
    config.image_budget_ms = 1;
    config.decoder_timeout_ms = 1;
    config.code128_direction_attempts = 1;
    decoder = xcr_decoder_create(&config);
    if (!decoder) {
        return 2;
    }

    {
        XcrBarcodeResult result;
        int result_count;
        (void)xcr_decoder_decode(decoder, &image, &result, 1, &result_count);
    }
    xcr_memory_stats_get(&before);
    for (index = 0; index < iterations; index++) {
        XcrBarcodeResult result;
        int result_count;
        XcrStatus status;

        if ((index & 1) != 0) {
            xcr_decoder_cancel(decoder);
        }
        status = xcr_decoder_decode(decoder, &image, &result, 1, &result_count);
        if ((index & 1) != 0) {
            xcr_decoder_clear_cancel(decoder);
            if (status != XCR_CANCELLED) {
                fprintf(stderr, "expected cancellation at %d: %s\n",
                        index, xcr_status_name(status));
                xcr_decoder_destroy(decoder);
                return 2;
            }
        } else if (status != XCR_NO_RESULT && status != XCR_TIMEOUT_NO_RESULT) {
            fprintf(stderr, "unexpected status at %d: %s\n", index, xcr_status_name(status));
            xcr_decoder_destroy(decoder);
            return 2;
        }
    }
    xcr_memory_stats_get(&after);
    printf("iterations=%d current_before=%zu current_after=%zu peak=%zu alloc=%u free=%u\n",
           iterations,
           before.current_bytes,
           after.current_bytes,
           after.peak_bytes,
           after.allocation_count,
           after.free_count);
    xcr_decoder_destroy(decoder);
    xcr_memory_stats_get(&after_destroy);
    printf("after_destroy=%zu\n", after_destroy.current_bytes);
    return before.current_bytes == after.current_bytes &&
           after_destroy.current_bytes == 0 ? 0 : 2;
}
