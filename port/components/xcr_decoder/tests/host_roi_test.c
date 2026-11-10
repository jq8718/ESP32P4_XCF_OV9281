#define _POSIX_C_SOURCE 200809L

#include "xcr_decoder.h"
#include "barcode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

static uint64_t monotonic_us(void)
{
#ifdef _WIN32
    static LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    if (frequency.QuadPart == 0) {
        QueryPerformanceFrequency(&frequency);
    }
    QueryPerformanceCounter(&counter);
    return (uint64_t)((counter.QuadPart * 1000000ULL) /
                      (uint64_t)frequency.QuadPart);
#else
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    return (uint64_t)timestamp.tv_sec * 1000000ULL +
           (uint64_t)timestamp.tv_nsec / 1000ULL;
#endif
}

int main(int argc, char **argv)
{
    Image bitmap = {0};
    XcrGrayImage image = {0};
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    XcrBarcodeResult result = {0};
    int result_count = 0;
    XcrStatus status;
    uint64_t started_us;

    if (argc != 6) {
        fprintf(stderr, "usage: %s <bmp> <roi_x> <roi_y> <roi_width> <roi_height>\n",
                argv[0]);
        return 2;
    }
    if (image_load_bmp(argv[1], &bitmap) != 0) {
        fprintf(stderr, "cannot load BMP: %s\n", argv[1]);
        return 2;
    }

    image.width = bitmap.width;
    image.height = bitmap.height;
    image.stride = bitmap.width;
    image.pixels = bitmap.pixels;
    image.roi_x = atoi(argv[2]);
    image.roi_y = atoi(argv[3]);
    image.roi_width = atoi(argv[4]);
    image.roi_height = atoi(argv[5]);

    xcr_decoder_config_default(&config);
    config.image_budget_ms = getenv("XCR_BUDGET_MS") ?
        (uint32_t)atoi(getenv("XCR_BUDGET_MS")) : 30000;
    config.decoder_timeout_ms = getenv("XCR_TIMEOUT_MS") ?
        (uint32_t)atoi(getenv("XCR_TIMEOUT_MS")) : 5000;
    if (getenv("XCR_NO_POLARITY_RETRY")) {
        config.polarity_retry = false;
    }
    if (getenv("XCR_DIRECTION_ATTEMPTS")) {
        config.code128_direction_attempts = (uint8_t)atoi(getenv("XCR_DIRECTION_ATTEMPTS"));
    }
    decoder = xcr_decoder_create(&config);
    if (!decoder) {
        image_free(&bitmap);
        fprintf(stderr, "decoder create failed\n");
        return 2;
    }

    started_us = monotonic_us();
    status = xcr_decoder_decode(decoder, &image, &result, 1, &result_count);
    printf("roi=%d,%d %dx%d status=%s results=%d elapsed_ms=%.3f",
           image.roi_x, image.roi_y, image.roi_width, image.roi_height,
           xcr_status_name(status), result_count,
           (double)(monotonic_us() - started_us) / 1000.0);
    if (result_count > 0) {
        printf(" payload=%s sym=%d conf=%d",
               result.data, result.symbology, result.confidence);
    }
    putchar('\n');

    xcr_decoder_destroy(decoder);
    image_free(&bitmap);
    return result_count > 0 ? 0 : 1;
}
