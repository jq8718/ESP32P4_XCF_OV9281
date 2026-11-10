#include "xcr_decoder.h"
#include "xcr_port.h"
#include "barcode.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    const char *name;
    const char *expected;
} QrCase;

static const QrCase k_cases[] = {
    {"gs1-qr.bmp", "GS1QR Code by XCtech"},
    {"microqr.bmp", "MicroQR 123"},
    {"microqr_inv.bmp", "InvMicroQR4"},
    {"microqr_mirror.bmp", "MirrorMicroQR"},
    {"qr-high.bmp", "HighQR Code by XCtech"},
    {"qr-low.bmp", "LowQR Code by XCtech"},
    {"qr-mid.bmp", "MidQR Code by XCtech"},
    {"qr-quar.bmp", "QuarQR Code by XCtech"},
    {"qr_inv.bmp", "InverseQR Code by XCtech"},
    {"qr_mirror.bmp", "MirrorQR Code by XCtech"},
};

int main(int argc, char **argv)
{
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    int passed = 0;

    if (argc != 2) {
        fprintf(stderr, "usage: %s <qr test directory>\n", argv[0]);
        return 2;
    }
    xcr_decoder_config_default(&config);
    config.enable_code128 = false;
    config.enable_upc_ean = false;
    config.enable_qr = true;
    config.polarity_retry = true;
    config.decoder_timeout_ms = 5000;
    config.image_budget_ms = 5000;
    decoder = xcr_decoder_create(&config);
    if (!decoder) {
        return 3;
    }

    for (size_t index = 0; index < sizeof(k_cases) / sizeof(k_cases[0]); index++) {
        char path[1024];
        Image image;
        XcrGrayImage input;
        XcrBarcodeResult result;
        int result_count = 0;
        uint64_t started;
        uint64_t elapsed;
        XcrStatus status;

        snprintf(path, sizeof(path), "%s/%s", argv[1], k_cases[index].name);
        memset(&image, 0, sizeof(image));
        if (image_load_bmp(path, &image) != 0) {
            printf("QR %-20s LOAD_FAIL\n", k_cases[index].name);
            continue;
        }
        memset(&input, 0, sizeof(input));
        input.width = image.width;
        input.height = image.height;
        input.stride = image.width;
        input.pixels = image.pixels;
        started = xcr_port_time_us();
        status = xcr_decoder_decode(decoder, &input, &result, 1, &result_count);
        elapsed = xcr_port_time_us() - started;
        printf("QR %-20s status=%s payload=%.*s match=%s time=%.3fms\n",
               k_cases[index].name, xcr_status_name(status),
               result_count > 0 ? result.length : 0,
               result_count > 0 ? result.data : "",
               result_count > 0 && strcmp(result.data, k_cases[index].expected) == 0
                   ? "YES" : "NO",
               (double)elapsed / 1000.0);
        if (status == XCR_OK && result_count == 1 &&
            strcmp(result.data, k_cases[index].expected) == 0) {
            passed++;
        }
        image_free(&image);
    }
    xcr_decoder_destroy(decoder);
    printf("QR SUMMARY %d/%zu passed\n", passed,
           sizeof(k_cases) / sizeof(k_cases[0]));
    return passed == (int)(sizeof(k_cases) / sizeof(k_cases[0])) ? 0 : 1;
}
