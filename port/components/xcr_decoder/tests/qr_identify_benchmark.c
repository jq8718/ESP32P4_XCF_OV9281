#include "barcode.h"
#include "xcr_decoder.h"
#include "xcr_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

int xcr_quirc_decode(const Image *image, BarcodeResult *result, XcrQrTrace *trace);
int xcr_quirc_decode_fast(const Image *image, BarcodeResult *result, XcrQrTrace *trace);
int xcr_quirc_decode_reference(const Image *image, BarcodeResult *result, XcrQrTrace *trace);

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t version, count, header_size, meta_size;
} BenchmarkHeader;

typedef struct __attribute__((packed)) {
    char name[32];
    uint8_t symbology;
    uint16_t width, height, stride;
    char expected[256];
    uint16_t expected_length;
    uint32_t offset, size;
} BenchmarkSample;

static int benchmark(const uint8_t *packed, size_t packed_size)
{
    BenchmarkHeader header;
    int regressions = 0;
    int baseline_success = 0;
    int optimized_success = 0;
    int fast_success = 0;
    int fast_regressions = 0;
    int api_success = 0;
    int api_regressions = 0;
    if (packed_size < sizeof(header)) return 2;
    memcpy(&header, packed, sizeof(header));
    if (memcmp(header.magic, "XCRS", 4) ||
        !((header.version == 1 && header.meta_size == 113) ||
          (header.version == 2 && header.meta_size == sizeof(BenchmarkSample))) ||
        header.count > 64 ||
        header.header_size != sizeof(header) ||
        sizeof(header) + header.count * header.meta_size > packed_size) return 2;
    for (uint32_t index = 0; index < header.count; index++) {
        BenchmarkSample sample;
        Image image = {0};
        BarcodeResult results[4] = {0};
        XcrQrTrace traces[4] = {0};
        int found[4] = {0};
        uint64_t times[4] = {0};
        const char *modes[] = {"baseline", "optimized", "fast", "api_fast"};
        const uint8_t *record = packed + sizeof(header) + index * header.meta_size;
        size_t expected_capacity = header.meta_size - 49;
        memset(&sample, 0, sizeof(sample));
        memcpy(&sample, record, 39);
        memcpy(sample.expected, record + 39, expected_capacity);
        memcpy(&sample.expected_length, record + 39 + expected_capacity, 10);
        if (sample.stride != sample.width || sample.size != (uint32_t)sample.width * sample.height ||
            sample.offset > packed_size || sample.size > packed_size - sample.offset ||
            sample.expected_length > sizeof(sample.expected)) return 2;
        image.width = sample.width;
        image.height = sample.height;
#ifdef ESP_PLATFORM
        image.pixels = heap_caps_malloc(sample.size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
        image.pixels = malloc(sample.size);
#endif
        if (!image.pixels) return 3;
        memcpy(image.pixels, packed + sample.offset, sample.size);
        for (int mode = 0; mode < 4; mode++) {
            uint64_t started = xcr_port_time_us();
            if (mode == 3) {
                XcrDecoderConfig config;
                xcr_decoder_config_default(&config);
                config.enable_code128 = false;
                config.enable_upc_ean = false;
                config.enable_qr = true;
                config.qr_quirc_only = true;
                config.qr_fast_search = true;
                XcrDecoder *decoder = xcr_decoder_create(&config);
                if (!decoder) return 3;
                XcrGrayImage gray = {.pixels = image.pixels, .width = image.width,
                                    .height = image.height, .stride = image.width};
                XcrBarcodeResult decoded = {0};
                int result_count = 0;
                xcr_decoder_decode(decoder, &gray, &decoded, 1, &result_count);
                found[mode] = result_count > 0;
                results[mode].length = decoded.length;
                memcpy(results[mode].data, decoded.data, sizeof(results[mode].data));
                xcr_decoder_get_qr_trace(decoder, &traces[mode]);
                printf("BENCH_GEOMETRY name=%.*s valid=%d refined=%u time_us=%lu corners=%d,%d,%d,%d,%d,%d,%d,%d\n",
                       32, sample.name, decoded.geometry_valid, traces[mode].geometry_refined,
                       (unsigned long)traces[mode].geometry_refine_us,
                       decoded.corners[0], decoded.corners[1], decoded.corners[2], decoded.corners[3],
                       decoded.corners[4], decoded.corners[5], decoded.corners[6], decoded.corners[7]);
                xcr_decoder_destroy(decoder);
            } else {
                found[mode] = mode == 0 ? xcr_quirc_decode_reference(&image, &results[mode], &traces[mode]) :
                              mode == 1 ? xcr_quirc_decode(&image, &results[mode], &traces[mode]) :
                                          xcr_quirc_decode_fast(&image, &results[mode], &traces[mode]);
            }
            times[mode] = xcr_port_time_us() - started;
            int match = found[mode] && results[mode].length == sample.expected_length &&
                        memcmp(results[mode].data, sample.expected, sample.expected_length) == 0;
            if (mode == 0) baseline_success += match;
            else if (mode == 1) optimized_success += match;
            else if (mode == 2) fast_success += match;
            else api_success += match;
            printf("BENCH name=%.*s mode=%s ok=%d match=%d total_us=%llu full8_identify=%lu full8_decode=%lu candidate_identify=%lu fallback_identify=%lu stage=%d payload=%s\n",
                   32, sample.name, modes[mode], found[mode], match,
                   (unsigned long long)times[mode], (unsigned long)traces[mode].full8_identify_us,
                   (unsigned long)traces[mode].full8_decode_us,
                   (unsigned long)traces[mode].candidate_identify_us,
                   (unsigned long)traces[mode].fallback_identify_us, (int)traces[mode].stage,
                   found[mode] ? results[mode].data : "");
            printf("BENCH_REFINE name=%.*s mode=%s full8=%lu candidate=%lu fallback=%lu finders=%u incomplete=%u\n",
                   32, sample.name, modes[mode],
                   (unsigned long)traces[mode].full8_refine_us,
                   (unsigned long)traces[mode].candidate_refine_us,
                   (unsigned long)traces[mode].fallback_refine_us, (unsigned)traces[mode].finder_count,
                   (unsigned)traces[mode].incomplete_count);
        }
        if (found[0] && (!found[1] || results[0].length != results[1].length ||
                        memcmp(results[0].data, results[1].data, results[0].length))) regressions++;
        if (found[1] && (!found[2] || results[1].length != results[2].length ||
                        memcmp(results[1].data, results[2].data, results[1].length))) fast_regressions++;
        if (found[2] != found[3] || (found[2] && (results[2].length != results[3].length ||
                memcmp(results[2].data, results[3].data, results[2].length)))) api_regressions++;
        XcrDecoderConfig bounded_config;
        xcr_decoder_config_default(&bounded_config);
        bounded_config.enable_code128 = false;
        bounded_config.enable_upc_ean = false;
        bounded_config.enable_qr = true;
        bounded_config.qr_quirc_only = true;
        bounded_config.qr_fast_search = true;
        bounded_config.polarity_retry = false;
        XcrDecoder *bounded = xcr_decoder_create(&bounded_config);
        if (!bounded) return 3;
        const uint32_t budgets[] = {0, 1000, 100000};
        for (unsigned attempt = 0; attempt < sizeof(budgets) / sizeof(budgets[0]); attempt++) {
            uint64_t begin = xcr_port_time_us();
            XcrGrayImage gray = {.pixels = image.pixels, .width = image.width,
                                .height = image.height, .stride = image.width,
                                .timestamp_us = begin, .deadline_us = begin + budgets[attempt],
                                .sequence = index + 1};
            XcrBarcodeResult result = {0};
            int count = 0;
            XcrStatus status = xcr_decoder_decode(bounded, &gray, &result, 1, &count);
            uint64_t elapsed = xcr_port_time_us() - begin;
            int invalid = count && (result.sequence != gray.sequence || result.timestamp_us != begin ||
                result.length != results[3].length || memcmp(result.data, results[3].data, result.length) ||
                status != XCR_OK || elapsed >= budgets[attempt]);
            if (attempt == 0 && (count || status != XCR_TIMEOUT_NO_RESULT)) invalid = 1;
            if (invalid) api_regressions++;
            printf("BENCH_DEADLINE name=%.*s budget=%lu elapsed=%llu count=%d status=%s invalid=%d\n",
                   32, sample.name, (unsigned long)budgets[attempt], (unsigned long long)elapsed,
                   count, xcr_status_name(status), invalid);
        }
        xcr_decoder_destroy(bounded);
        free(image.pixels);
#ifdef ESP_PLATFORM
        vTaskDelay(1);
#endif
    }
    printf("BENCH SUMMARY samples=%lu baseline=%d optimized=%d regressions=%d\n",
           (unsigned long)header.count, baseline_success, optimized_success, regressions);
    printf("BENCH FAST success=%d regressions=%d\n", fast_success, fast_regressions);
    printf("BENCH API success=%d regressions=%d\n", api_success, api_regressions);
    return regressions || fast_regressions || api_regressions ? 1 : 0;
}

#ifdef ESP_PLATFORM
static void benchmark_task(void *argument)
{
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                               ESP_PARTITION_SUBTYPE_ANY, "samples");
    const void *packed;
    esp_partition_mmap_handle_t mapping;
    if (!partition || esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA,
                                        &packed, &mapping) != ESP_OK) {
        printf("BENCH partition error\n");
        vTaskDelete(NULL);
        return;
    }
    printf("BENCH exit=%d\n", benchmark(packed, partition->size));
    esp_partition_munmap(mapping);
    vTaskDelete(NULL);
}

void app_main(void)
{
    if (xTaskCreatePinnedToCore(benchmark_task, "qr_benchmark", 32768, NULL, 5, NULL, 1) != pdPASS)
        printf("BENCH task allocation failed\n");
}
#else
int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    FILE *input = fopen(argv[1], "rb");
    if (!input) return 2;
    fseek(input, 0, SEEK_END);
    long size = ftell(input);
    rewind(input);
    if (size <= 0) { fclose(input); return 2; }
    uint8_t *packed = malloc((size_t)size);
    if (!packed) { fclose(input); return 3; }
    if (fread(packed, 1, (size_t)size, input) != (size_t)size) {
        free(packed); fclose(input); return 2;
    }
    fclose(input);
    int result = benchmark(packed, (size_t)size);
    free(packed);
    return result;
}
#endif
