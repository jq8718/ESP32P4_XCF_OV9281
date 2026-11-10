#include "barcode.h"
#include "datamatrix.h"
#include "xcr_dmtx.h"
#include "xcr_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

int xcr_datamatrix_marked_decode_ex(const Image *image, BarcodeResult *result,
                                    const XcrDeadline *deadline, int defer_geometry);
int xcr_datamatrix_marked_decode_prepared(const Image *image, BarcodeResult *result,
    const XcrDeadline *deadline, int defer_geometry, const XcrDmtxPrepared *prepared);

static int trace_file(const char *path)
{
    const int width = 1280, height = 720;
    uint8_t *pixels = malloc((size_t)width * height);
    if (!pixels) return 2;
    FILE *input = fopen(path, "rb");
    if (!input || fread(pixels, 1, (size_t)width * height, input) != (size_t)width * height) {
        fprintf(stderr, "unable to read 1280x720 Gray8 frame\n");
        if (input) fclose(input);
        free(pixels);
        return 2;
    }
    fclose(input);
    Image image = {.width = width, .height = height, .pixels = pixels};
    BarcodeResult result = {0};
    uint64_t start = xcr_port_time_us();
    int decoded;
    int repeats = getenv("XCR_DM_TRACE_REPEATS") ? atoi(getenv("XCR_DM_TRACE_REPEATS")) : 1;
    if (repeats < 1 || repeats > 64) return 2;
    for (int iteration = 0; iteration < repeats; iteration++) {
    if (getenv("XCR_DM_PREPARED")) {
        XcrGrayImage source = {.pixels = pixels, .width = width, .height = height,
                               .stride = width, .sequence = 1};
        XcrDmtxPrepared prepared;
        if (!xcr_dmtx_prepare(&source, &prepared)) return 2;
        decoded = xcr_datamatrix_marked_decode_prepared(&image, &result, NULL, 0, &prepared);
    } else {
        decoded = xcr_datamatrix_marked_decode_ex(&image, &result, NULL, 0);
    }
    }
    uint64_t elapsed = xcr_port_time_us() - start;
    XcrDmtxStats marker = {0};
    DataMatrixDecodeDebugStats core = {0};
    xcr_dmtx_stats_get(&marker);
    datamatrix_debug_stats_get(&core);
    printf("{\"iterations\":%d,\"decoded\":%d,\"payload\":\"%s\",\"elapsed_us\":%llu,"
           "\"marker_us\":%llu,\"normalize_us\":%llu,\"core_us\":%llu,"
           "\"refine_us\":%llu,\"marker_roi\":[%d,%d,%d,%d],"
           "\"angle_deg\":%.3f,\"grid\":[%d,%d],\"geometry_quality\":%d,"
           "\"split\":%u,\"full_fallback\":%u,\"sample_step\":%u,"
           "\"core_finder_us\":%llu,\"core_candidate_us\":%llu,"
           "\"core_codeword_us\":%llu,\"core_ecc_us\":%llu,\"core_payload_us\":%llu,"
           "\"corners\":[%d,%d,%d,%d,%d,%d,%d,%d]}\n",
           repeats, decoded, result.data, (unsigned long long)elapsed,
           (unsigned long long)marker.prescreen_us, (unsigned long long)marker.setup_us,
           (unsigned long long)marker.decode_us, (unsigned long long)marker.geometry_us,
           marker.marker_roi[0], marker.marker_roi[1], marker.marker_roi[2], marker.marker_roi[3],
           marker.angle_degrees, result.grid_rows, result.grid_cols, result.geometry_quality,
           marker.marker_parallel_used, marker.marker_full_fallback, marker.marker_sample_step,
           (unsigned long long)core.finder_us, (unsigned long long)core.candidate_us,
           (unsigned long long)core.codeword_us, (unsigned long long)core.ecc_us,
           (unsigned long long)core.payload_us,
           result.corners[0], result.corners[1], result.corners[2], result.corners[3],
           result.corners[4], result.corners[5], result.corners[6], result.corners[7]);
    free(pixels);
    return decoded > 0 ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s source.gray8 [source2.gray8 ...]\n", argv[0]);
        return 2;
    }
    int status = 0;
    for (int index = 1; index < argc; index++) {
        int result = trace_file(argv[index]);
        if (result > status) status = result;
    }
    return status;
}
