#define _POSIX_C_SOURCE 200809L

#include "xcr_decoder.h"
#include "barcode.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint64_t monotonic_us(void)
{
    struct timespec timestamp;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    return (uint64_t)timestamp.tv_sec * 1000000ULL +
           (uint64_t)timestamp.tv_nsec / 1000ULL;
}

int main(int argc, char **argv)
{
    FILE *manifest;
    char line[2048];
    int total = 0;
    int passed = 0;
    uint64_t total_us = 0;
    uint64_t max_us = 0;
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    uint32_t budget_ms = getenv("XCR_BUDGET_MS") ?
        (uint32_t)atoi(getenv("XCR_BUDGET_MS")) : 120;

    if (argc != 3) {
        fprintf(stderr, "usage: %s <manifest.tsv> <image-root>\n", argv[0]);
        return 2;
    }
    manifest = fopen(argv[1], "r");
    if (!manifest) {
        perror(argv[1]);
        return 2;
    }
    xcr_decoder_config_default(&config);
    config.enable_upc_ean = false;
    config.polarity_retry = false;
    config.cross_symbology_veto = false;
    config.code128_direction_attempts = 1;
    config.code128_fast_rows = true;
    config.code128_fast_rows_only = true;
    config.decoder_timeout_ms = budget_ms;
    config.image_budget_ms = budget_ms;
    decoder = xcr_decoder_create(&config);
    if (!decoder) {
        fclose(manifest);
        return 2;
    }

    while (fgets(line, sizeof(line), manifest)) {
        char *type;
        char *path_text;
        char *expected;
        char path[4096];
        Image bitmap = {0};
        XcrGrayImage image = {0};
        XcrBarcodeResult result = {0};
        int result_count = 0;
        XcrStatus status;
        uint64_t started;
        uint64_t elapsed;
        char *tab;

        line[strcspn(line, "\r\n")] = '\0';
        type = line;
        tab = strchr(type, '\t');
        if (!tab) continue;
        *tab++ = '\0';
        path_text = tab;
        tab = strchr(path_text, '\t');
        if (!tab) continue;
        *tab++ = '\0';
        expected = tab;
        if (strcmp(type, "code128") != 0) continue;
        if (snprintf(path, sizeof(path), "%s/%s", argv[2], path_text) >=
            (int)sizeof(path)) {
            continue;
        }
        if (image_load_bmp(path, &bitmap) != 0) {
            fprintf(stderr, "LOAD_FAIL %s\n", path_text);
            continue;
        }
        image.width = bitmap.width;
        image.height = bitmap.height;
        image.stride = bitmap.width;
        image.pixels = bitmap.pixels;
        started = monotonic_us();
        status = xcr_decoder_decode(decoder, &image, &result, 1,
                                     &result_count);
        elapsed = monotonic_us() - started;
        total++;
        total_us += elapsed;
        if (elapsed > max_us) max_us = elapsed;
        if (status != XCR_OOM && result_count > 0 &&
            result.symbology == XCR_SYM_CODE128 &&
            strcmp(result.data, expected) == 0) {
            passed++;
            printf("OK %s %.3f ms %s\n", path_text,
                   (double)elapsed / 1000.0, result.data);
        } else {
            printf("MISS %s %.3f ms status=%s results=%d payload=%s\n",
                   path_text, (double)elapsed / 1000.0,
                   xcr_status_name(status), result_count,
                   result_count > 0 ? result.data : "");
        }
        image_free(&bitmap);
    }
    fclose(manifest);
    xcr_decoder_destroy(decoder);
    printf("TOTAL=%d PASSED=%d FAILED=%d AVG=%.3f ms MAX=%.3f ms\n",
           total, passed, total - passed,
           total ? (double)total_us / (double)total / 1000.0 : 0.0,
           (double)max_us / 1000.0);
    return total == passed ? 0 : 1;
}
