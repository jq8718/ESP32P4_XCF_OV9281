#include "xcr_decoder.h"
#include "xcr_port.h"
#include "barcode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_CAPACITY 2048
#define PATH_CAPACITY 2048

static int split_tsv(char *line, char **fields, int capacity)
{
    int count = 0;
    char *cursor = line;

    while (count < capacity) {
        char *tab;
        fields[count++] = cursor;
        tab = strchr(cursor, '\t');
        if (!tab) {
            break;
        }
        *tab = '\0';
        cursor = tab + 1;
    }
    if (count > 0) {
        fields[count - 1][strcspn(fields[count - 1], "\r\n")] = '\0';
    }
    return count;
}

static int expected_symbology(const char *type, const char *file)
{
    if (strcmp(type, "code128") == 0 || strcmp(type, "isbt128") == 0) {
        return XCR_SYM_CODE128;
    }
    if (strcmp(type, "ean13") == 0 || strcmp(type, "ean8") == 0 ||
        strcmp(type, "upca") == 0 || strcmp(type, "upce") == 0 ||
        strcmp(type, "isbn") == 0) {
        return XCR_SYM_UPC_EAN;
    }
    if (strcmp(type, "gs1") == 0 && strstr(file, "gs1-128") != NULL &&
        strstr(file, "cca") == NULL) {
        return XCR_SYM_CODE128;
    }
    return XCR_SYM_UNKNOWN;
}

int main(int argc, char **argv)
{
    const char *manifest_path;
    const char *image_root;
    FILE *manifest;
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    char line[LINE_CAPACITY];
    int total = 0;
    int correct = 0;
    int missed = 0;
    int wrong = 0;
    int unsupported_clean = 0;

    if (argc != 3) {
        fprintf(stderr, "usage: %s <manifest.tsv> <image-root>\n", argv[0]);
        return 2;
    }
    manifest_path = argv[1];
    image_root = argv[2];
    manifest = fopen(manifest_path, "r");
    if (!manifest) {
        perror(manifest_path);
        return 2;
    }

    xcr_decoder_config_default(&config);
    decoder = xcr_decoder_create(&config);
    if (!decoder) {
        fclose(manifest);
        fprintf(stderr, "decoder create failed\n");
        return 2;
    }

    if (!fgets(line, sizeof(line), manifest)) {
        fclose(manifest);
        xcr_decoder_destroy(decoder);
        return 2;
    }
    while (fgets(line, sizeof(line), manifest)) {
        char *fields[4];
        int field_count = split_tsv(line, fields, 4);
        char full_path[PATH_CAPACITY];
        Image bitmap = {0};
        XcrGrayImage input = {0};
        XcrBarcodeResult result = {0};
        XcrStatus status;
        int result_count = 0;
        int expected_sym;
        int row_ok = 0;

        if (field_count < 3 || fields[0][0] == '\0' || fields[1][0] == '\0') {
            continue;
        }
        if (snprintf(full_path, sizeof(full_path), "%s/%s", image_root, fields[1]) >=
            (int)sizeof(full_path)) {
            fprintf(stderr, "path too long: %s\n", fields[1]);
            wrong++;
            continue;
        }
        if (image_load_bmp(full_path, &bitmap) != 0) {
            fprintf(stderr, "cannot load: %s\n", full_path);
            wrong++;
            continue;
        }

        input.width = bitmap.width;
        input.height = bitmap.height;
        input.stride = bitmap.width;
        input.pixels = bitmap.pixels;
        status = xcr_decoder_decode(decoder, &input, &result, 1, &result_count);
        expected_sym = expected_symbology(fields[0], fields[1]);
        total++;

        if (expected_sym == XCR_SYM_UNKNOWN) {
            if (result_count == 0) {
                unsupported_clean++;
                row_ok = 1;
            }
        } else if (result_count > 0 && result.symbology == expected_sym &&
                   strcmp(result.data, fields[2]) == 0) {
            correct++;
            row_ok = 1;
        } else if (result_count == 0) {
            missed++;
        }

        if (!row_ok && result_count > 0) {
            wrong++;
        }
        printf("%-5s %-10s %-16s expected=%s got=%s sym=%d conf=%d status=%s\n",
               row_ok ? "OK" : (result_count > 0 ? "WRONG" : "MISS"),
               fields[0],
               fields[1],
               fields[2],
               result_count > 0 ? result.data : "",
               result_count > 0 ? result.symbology : 0,
               result_count > 0 ? result.confidence : 0,
               xcr_status_name(status));
        image_free(&bitmap);
    }

    fclose(manifest);
    xcr_decoder_destroy(decoder);
    printf("TOTAL=%d CORRECT=%d UNSUPPORTED_CLEAN=%d WRONG=%d MISS=%d\n",
           total, correct, unsupported_clean, wrong, missed);
    return (wrong == 0 && missed == 0) ? 0 : 2;
}
