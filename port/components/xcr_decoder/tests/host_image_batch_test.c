#define _POSIX_C_SOURCE 200809L

#include "xcr_decoder.h"
#include "barcode.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define LINE_CAPACITY 2048
#define PATH_CAPACITY 2048
#define MAX_SAMPLES 64
#define ROI_HEIGHT 256
#define ROW_STEP 16
#define ORIENTATION_STEP 8
#define MAX_CANDIDATES 32
#define MAX_TRIAL_CANDIDATES 24

static const int s_slopes[] = {
    -65536, -55013, -49152, -37837, -32768, -26412,
    -22938, -17561, -11585, -5730, -4096, -2048, -1024, 0,
    1024, 2048, 4096, 5730, 11585, 17561, 22938, 26412, 32768, 37837,
    49152, 55013, 65536,
};

typedef struct {
    int quarter_turns;
    int angle_deg;
    int fallback;
    int y;
    int line_y;
    int slope_q16;
    uint32_t score;
} candidate_t;

static const int s_fallback_angles[] = {
    -75, -60, -45, -30, 30, 45, 60, 75,
};

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

static void rotate_quarter(const uint8_t *source, uint8_t *destination,
                           int width, int height, int quarter_turns)
{
    int x;
    int y;
    quarter_turns &= 3;
    if (quarter_turns == 0) {
        memcpy(destination, source, (size_t)width * (size_t)height);
        return;
    }
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int source_x;
            int source_y;
            if (quarter_turns == 1) {
                source_x = y;
                source_y = height - 1 - x;
            } else if (quarter_turns == 2) {
                source_x = width - 1 - x;
                source_y = height - 1 - y;
            } else {
                source_x = width - 1 - y;
                source_y = x;
            }
            destination[(size_t)y * (size_t)width + (size_t)x] =
                source[(size_t)source_y * (size_t)width + (size_t)source_x];
        }
    }
}

static void rotate_angle(const uint8_t *source, uint8_t *destination,
                         int width, int height, int angle_deg)
{
    double radians = (double)angle_deg * 3.14159265358979323846 / 180.0;
    double cosine = cos(radians);
    double sine = sin(radians);
    double center_x = (double)width * 0.5;
    double center_y = (double)height * 0.5;
    int y;
    for (y = 0; y < height; y++) {
        int x;
        for (x = 0; x < width; x++) {
            double dx = (double)x - center_x;
            double dy = (double)y - center_y;
            double source_x_f = cosine * dx - sine * dy + center_x;
            double source_y_f = sine * dx + cosine * dy + center_y;
            int source_x = (int)floor(source_x_f);
            int source_y = (int)floor(source_y_f);
            if (source_x < 0 || source_x + 1 >= width ||
                source_y < 0 || source_y + 1 >= height) {
                destination[(size_t)y * (size_t)width + (size_t)x] = 255;
            } else {
                double fx = source_x_f - (double)source_x;
                double fy = source_y_f - (double)source_y;
                double top = (double)source[(size_t)source_y * (size_t)width +
                                             (size_t)source_x] * (1.0 - fx) +
                    (double)source[(size_t)source_y * (size_t)width +
                                    (size_t)source_x + 1] * fx;
                double bottom =
                    (double)source[(size_t)(source_y + 1) * (size_t)width +
                                    (size_t)source_x] * (1.0 - fx) +
                    (double)source[(size_t)(source_y + 1) * (size_t)width +
                                    (size_t)source_x + 1] * fx;
                destination[(size_t)y * (size_t)width + (size_t)x] =
                    (uint8_t)llround(top * (1.0 - fy) + bottom * fy);
            }
        }
    }
}

static uint32_t row_score(const uint8_t *pixels, int width, int row)
{
    uint32_t score = 0;
    int x;
    const uint8_t *line = pixels + (size_t)row * (size_t)width;
    for (x = 4; x < width; x += 4) {
        int delta = (int)line[x] - (int)line[x - 4];
        score += (uint32_t)(delta < 0 ? -delta : delta);
    }
    return score;
}

static void find_orientation(const uint8_t *pixels, int width, int height,
                             int roi_y, int *out_slope, int *out_line_y)
{
    int best_slope = 0;
    int best_line_y = roi_y + ROI_HEIGHT / 2;
    int best_score = 0;
    int horizontal_score = 0;
    int midpoint = (width - 1) / 2;
    size_t slope_index;

    for (slope_index = 0; slope_index < sizeof(s_slopes) / sizeof(s_slopes[0]);
         slope_index++) {
        int slope = s_slopes[slope_index];
        int slope_best = 0;
        int slope_line = best_line_y;
        int first_line = roi_y - ROI_HEIGHT / 2;
        int last_line = roi_y + ROI_HEIGHT + ROI_HEIGHT / 2;
        int line_y;

        if (first_line < 0) first_line = 0;
        if (last_line >= height) last_line = height - 1;
        for (line_y = first_line; line_y <= last_line; line_y += ORIENTATION_STEP) {
            uint32_t sum = 0;
            int samples = 0;
            int previous = -1;
            int x;
            for (x = 4; x < width; x += 4) {
                int y = line_y + (int)(((long long)(x - midpoint) * slope) >> 16);
                if (y < 0 || y >= height) {
                    previous = -1;
                    continue;
                }
                {
                    int value = pixels[(size_t)y * (size_t)width + (size_t)x];
                    if (previous >= 0) {
                        int delta = value - previous;
                        sum += (uint32_t)(delta < 0 ? -delta : delta);
                        samples++;
                    }
                    previous = value;
                }
            }
            if (samples >= width / 8) {
                int score = (int)((sum * 1000U) / (uint32_t)samples);
                if (score > slope_best) {
                    slope_best = score;
                    slope_line = line_y;
                }
            }
        }
        if (slope == 0) horizontal_score = slope_best;
        if (slope_best > best_score) {
            best_score = slope_best;
            best_slope = slope;
            best_line_y = slope_line;
        }
    }
    if (best_slope == 0 || horizontal_score <= 0 ||
        best_score < horizontal_score + horizontal_score / 50) {
        best_slope = 0;
    }
    *out_slope = best_slope;
    *out_line_y = best_line_y;
}

static void build_roi(const uint8_t *pixels, int width, int height,
                      const candidate_t *candidate, uint8_t *output)
{
    int midpoint = (width - 1) / 2;
    int output_y;
    for (output_y = 0; output_y < ROI_HEIGHT; output_y++) {
        int offset_y = output_y - ROI_HEIGHT / 2;
        int x;
        for (x = 0; x < width; x++) {
            int source_y = candidate->line_y + offset_y +
                (int)(((long long)(x - midpoint) * candidate->slope_q16) >> 16);
            if (source_y < 0 || source_y >= height) {
                output[(size_t)output_y * (size_t)width + (size_t)x] = 255;
            } else {
                output[(size_t)output_y * (size_t)width + (size_t)x] =
                    pixels[(size_t)source_y * (size_t)width + (size_t)x];
            }
        }
    }
}

static void scale_horizontal(const uint8_t *input, uint8_t *output,
                             int width, int height)
{
    int y;
    for (y = 0; y < height; y++) {
        const uint8_t *source = input + (size_t)y * (size_t)width;
        uint8_t *destination = output + (size_t)y * (size_t)(width * 2);
        int x;
        for (x = 0; x < width; x++) {
            destination[x * 2] = source[x];
            destination[x * 2 + 1] = source[x];
        }
    }
}

static int write_gray_bmp(const char *path, const uint8_t *pixels,
                          int width, int height)
{
    FILE *file;
    uint8_t header[54] = {0};
    int row;
    size_t row_size = ((size_t)width + 3U) & ~3U;
    size_t data_size = row_size * (size_t)height;
    uint8_t *row_buffer;

    if (!path || !pixels || width <= 0 || height <= 0) return -1;
    file = fopen(path, "wb");
    if (!file) return -1;
    row_buffer = (uint8_t *)calloc(row_size, 1);
    if (!row_buffer) {
        fclose(file);
        return -1;
    }
    header[0] = 'B';
    header[1] = 'M';
    *(uint32_t *)&header[2] = (uint32_t)(54U + 1024U + data_size);
    *(uint32_t *)&header[10] = 54U + 1024U;
    *(uint32_t *)&header[14] = 40U;
    *(int32_t *)&header[18] = width;
    *(int32_t *)&header[22] = height;
    *(uint16_t *)&header[26] = 1U;
    *(uint16_t *)&header[28] = 8U;
    *(uint32_t *)&header[34] = (uint32_t)data_size;
    if (fwrite(header, 1, sizeof(header), file) != sizeof(header)) {
        free(row_buffer);
        fclose(file);
        return -1;
    }
    for (row = 0; row < 256; row++) {
        uint8_t gray[4] = {0};
        gray[0] = (uint8_t)row;
        gray[1] = (uint8_t)row;
        gray[2] = (uint8_t)row;
        if (fwrite(gray, 1, sizeof(gray), file) != sizeof(gray)) {
            free(row_buffer);
            fclose(file);
            return -1;
        }
    }
    for (row = height - 1; row >= 0; row--) {
        memset(row_buffer, 0, row_size);
        memcpy(row_buffer, pixels + (size_t)row * (size_t)width,
               (size_t)width);
        if (fwrite(row_buffer, 1, row_size, file) != row_size) {
            free(row_buffer);
            fclose(file);
            return -1;
        }
    }
    free(row_buffer);
    fclose(file);
    return 0;
}

static int enhance(const uint8_t *input, uint8_t *output, size_t count)
{
    uint32_t histogram[256] = {0};
    size_t index;
    size_t low_target = count / 100U;
    size_t high_target = count - count / 100U;
    uint32_t cumulative = 0;
    int low = 0;
    int high = 255;
    int range;

    if (low_target == 0) low_target = 1;
    for (index = 0; index < count; index++) histogram[input[index]]++;
    for (index = 0; index < 256; index++) {
        cumulative += histogram[index];
        if (cumulative >= low_target) { low = (int)index; break; }
    }
    cumulative = 0;
    for (index = 0; index < 256; index++) {
        cumulative += histogram[index];
        if (cumulative >= high_target) { high = (int)index; break; }
    }
    range = high - low;
    if (range < 16) return 0;
    for (index = 0; index < count; index++) {
        int value = ((int)input[index] - low) * 255 / range;
        if (value < 0) value = 0;
        if (value > 255) value = 255;
        output[index] = (uint8_t)value;
    }
    return 1;
}

static void enhance_rows(const uint8_t *input, uint8_t *output,
                         int width, int height, int binary)
{
    int y;
    for (y = 0; y < height; y++) {
        uint32_t histogram[256] = {0};
        const uint8_t *source = input + (size_t)y * (size_t)width;
        uint8_t *destination = output + (size_t)y * (size_t)width;
        int low = 0;
        int high = 255;
        int threshold;
        uint32_t cumulative = 0;
        int x;

        for (x = 0; x < width; x++) histogram[source[x]]++;
        for (x = 0; x < 256; x++) {
            cumulative += histogram[x];
            if (cumulative >= (uint32_t)(width / 20)) { low = x; break; }
        }
        cumulative = 0;
        for (x = 0; x < 256; x++) {
            cumulative += histogram[x];
            if (cumulative >= (uint32_t)(width - width / 20)) { high = x; break; }
        }
        if (high <= low) {
            memcpy(destination, source, (size_t)width);
            continue;
        }
        threshold = (low + high) / 2;
        for (x = 0; x < width; x++) {
            if (binary) {
                destination[x] = source[x] < threshold ? 0 : 255;
            } else {
                int value = ((int)source[x] - low) * 255 / (high - low);
                if (value < 0) value = 0;
                if (value > 255) value = 255;
                destination[x] = (uint8_t)value;
            }
        }
    }
}

static void enhance_local(const uint8_t *input, uint8_t *output,
                          int width, int height, int binary)
{
    uint32_t prefix[1025];
    int y;
    for (y = 0; y < height; y++) {
        const uint8_t *source = input + (size_t)y * (size_t)width;
        uint8_t *destination = output + (size_t)y * (size_t)width;
        int x;
        prefix[0] = 0;
        for (x = 0; x < width; x++) prefix[x + 1] = prefix[x] + source[x];
        for (x = 0; x < width; x++) {
            int left = x - 16;
            int right = x + 16;
            int mean;
            int value;
            if (left < 0) left = 0;
            if (right >= width) right = width - 1;
            mean = (int)((prefix[right + 1] - prefix[left]) / (uint32_t)(right - left + 1));
            if (binary) {
                destination[x] = source[x] + 2 < mean ? 0 : 255;
            } else {
                value = 128 + ((int)source[x] - mean) * 8;
                if (value < 0) value = 0;
                if (value > 255) value = 255;
                destination[x] = (uint8_t)value;
            }
        }
    }
}

static void enhance_2d(const uint8_t *input, uint8_t *output,
                       uint16_t *horizontal_mean, int width, int height,
                       int radius, int gain)
{
    int y;
    for (y = 0; y < height; y++) {
        const uint8_t *source = input + (size_t)y * (size_t)width;
        uint16_t *mean_row = horizontal_mean + (size_t)y * (size_t)width;
        int left = 0;
        int right = radius < width ? radius : width - 1;
        uint32_t sum = 0;
        int x;
        for (x = left; x <= right; x++) sum += source[x];
        for (x = 0; x < width; x++) {
            mean_row[x] = (uint16_t)(sum / (uint32_t)(right - left + 1));
            if (x == width - 1) break;
            if (x - radius >= 0) sum -= source[x - radius];
            if (x + radius + 1 < width) sum += source[x + radius + 1];
            left = x + 1 - radius;
            if (left < 0) left = 0;
            right = x + 1 + radius;
            if (right >= width) right = width - 1;
        }
    }
    for (y = 0; y < height; y++) {
        int top = y - radius;
        int bottom = y + radius;
        int x;
        if (top < 0) top = 0;
        if (bottom >= height) bottom = height - 1;
        for (x = 0; x < width; x++) {
            const uint8_t *source = input + (size_t)y * (size_t)width;
            uint8_t *destination = output + (size_t)y * (size_t)width;
            uint32_t sum = 0;
            int row;
            for (row = top; row <= bottom; row++) {
                sum += horizontal_mean[(size_t)row * (size_t)width +
                                       (size_t)x];
            }
            int mean = (int)(sum / (uint32_t)(bottom - top + 1));
            int value = 128 + ((int)source[x] - mean) * gain;
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            destination[x] = (uint8_t)value;
            if (x == width - 1) break;
        }
    }
}

static int candidate_compare(const void *a, const void *b)
{
    const candidate_t *left = (const candidate_t *)a;
    const candidate_t *right = (const candidate_t *)b;
    if (left->fallback != right->fallback) {
        return left->fallback ? 1 : -1;
    }
    return right->score > left->score ? 1 : right->score < left->score ? -1 : 0;
}

static int expected_match(const XcrBarcodeResult *result, int result_count,
                          const char *expected)
{
    return result_count > 0 && result->symbology == XCR_SYM_CODE128 &&
           strcmp(result->data, expected) == 0;
}

int main(int argc, char **argv)
{
    FILE *manifest;
    char line[LINE_CAPACITY];
    XcrDecoderConfig config;
    XcrDecoder *decoder;
    XcrDecoder *fallback_decoder;
    const char *only = getenv("XCR_ONLY");
    const char *trace_variants = getenv("XCR_TRACE_VARIANTS");
    const char *dump_roi_dir = getenv("XCR_DUMP_ROI_DIR");
    const char *fallback_angle_text = getenv("XCR_FALLBACK_ANGLE");
    int total = 0;
    int passed = 0;
    int failed = 0;

    if (argc != 3) {
        fprintf(stderr, "usage: %s <manifest.tsv> <image-root>\n", argv[0]);
        return 2;
    }
    manifest = fopen(argv[1], "r");
    if (!manifest) { perror(argv[1]); return 2; }
    xcr_decoder_config_default(&config);
    config.decoder_timeout_ms = getenv("XCR_BUDGET_MS") ?
        (uint32_t)atoi(getenv("XCR_BUDGET_MS")) : 1200;
    config.image_budget_ms = getenv("XCR_BUDGET_MS") ?
        (uint32_t)atoi(getenv("XCR_BUDGET_MS")) : 1200;
    config.code128_direction_attempts = 2;
    config.code128_fast_rows = getenv("XCR_FAST_ROWS") != NULL;
    config.code128_fast_rows_only = getenv("XCR_FAST_ROWS_ONLY") != NULL;
    decoder = xcr_decoder_create(&config);
    if (!decoder) { fclose(manifest); return 2; }
    {
        XcrDecoderConfig fallback_config = config;
        if (!getenv("XCR_BUDGET_MS")) {
            fallback_config.decoder_timeout_ms = 5000;
            fallback_config.image_budget_ms = 5000;
        }
        fallback_decoder = xcr_decoder_create(&fallback_config);
    }
    if (!fallback_decoder) {
        xcr_decoder_destroy(decoder);
        fclose(manifest);
        return 2;
    }

    while (fgets(line, sizeof(line), manifest)) {
        char *fields[4];
        char path[PATH_CAPACITY];
        Image bitmap = {0};
        uint8_t *rotated;
        uint8_t *roi;
        uint8_t *enhanced_roi;
        uint8_t *row_enhanced_roi;
        uint8_t *binary_roi;
        uint8_t *local_roi;
        uint8_t *local_binary_roi;
        uint8_t *enhanced_2d_roi;
        uint16_t *horizontal_mean;
        uint8_t *scaled_roi;
        uint8_t *scaled_enhanced_roi;
        candidate_t candidates[MAX_CANDIDATES];
        int candidate_count = 0;
        int field_count = split_tsv(line, fields, 4);
        int quarter;
        int found = 0;
        int variant;
        uint64_t started;
        uint64_t elapsed;

        if (field_count < 3 || fields[0][0] == '\0') continue;
        if (only && !strstr(fields[1], only)) continue;
        if (snprintf(path, sizeof(path), "%s/%s", argv[2], fields[1]) >=
            (int)sizeof(path)) { failed++; continue; }
        if (image_load_bmp(path, &bitmap) != 0) { failed++; continue; }
        if (bitmap.width != 1024 || bitmap.height != 1024) {
            fprintf(stderr, "SKIP %s: expected 1024x1024\n", fields[1]);
            image_free(&bitmap); failed++; continue;
        }
        rotated = malloc((size_t)bitmap.width * (size_t)bitmap.height);
        roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        enhanced_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        row_enhanced_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        binary_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        local_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        local_binary_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        enhanced_2d_roi = malloc((size_t)bitmap.width * ROI_HEIGHT);
        horizontal_mean = malloc((size_t)bitmap.width * ROI_HEIGHT *
                                  sizeof(*horizontal_mean));
        scaled_roi = malloc((size_t)bitmap.width * 2U * ROI_HEIGHT);
        scaled_enhanced_roi = malloc((size_t)bitmap.width * 2U * ROI_HEIGHT);
        if (!rotated || !roi || !enhanced_roi || !row_enhanced_roi ||
            !binary_roi || !local_roi || !local_binary_roi ||
            !enhanced_2d_roi || !horizontal_mean ||
            !scaled_roi || !scaled_enhanced_roi) {
            free(rotated); free(roi); free(enhanced_roi);
            free(row_enhanced_roi); free(binary_roi); free(local_roi);
            free(local_binary_roi);
            free(enhanced_2d_roi); free(horizontal_mean);
            free(scaled_roi); free(scaled_enhanced_roi);
            image_free(&bitmap); return 2;
        }
        for (quarter = 0; quarter < 4; quarter++) {
            uint32_t row_scores[1024];
            int selected[1024] = {0};
            int selected_count = 0;
            int y;
            rotate_quarter(bitmap.pixels, rotated, bitmap.width, bitmap.height, quarter);
            for (y = 0; y < bitmap.height; y++) row_scores[y] = row_score(rotated, bitmap.width, y);
            while (selected_count < 3 && candidate_count < MAX_CANDIDATES) {
                candidate_t candidate = {0};
                uint32_t score = 0;
                int best_y = 0;
                for (y = 0; y <= bitmap.height - ROI_HEIGHT; y += ROW_STEP) {
                    uint32_t band_score = 0;
                    int row;
                    if (selected[y]) continue;
                    for (row = y; row < y + ROI_HEIGHT; row++) band_score += row_scores[row];
                    if (band_score > score) { score = band_score; best_y = y; }
                }
                candidate.quarter_turns = quarter;
                candidate.score = score;
                candidate.y = best_y;
                candidate.line_y = best_y + ROI_HEIGHT / 2;
                find_orientation(rotated, bitmap.width, bitmap.height, candidate.y,
                                 &candidate.slope_q16, &candidate.line_y);
                candidates[candidate_count++] = candidate;
                selected_count++;
                for (y = best_y - ROI_HEIGHT; y < best_y + ROI_HEIGHT * 2; y += ROW_STEP) {
                    if (y >= 0 && y <= bitmap.height - ROI_HEIGHT) selected[y] = 1;
                }
            }
        }
        for (quarter = 0;
             quarter < (int)(sizeof(s_fallback_angles) /
                             sizeof(s_fallback_angles[0])) &&
             candidate_count < MAX_CANDIDATES;
             quarter++) {
            if (fallback_angle_text &&
                atoi(fallback_angle_text) != s_fallback_angles[quarter]) {
                continue;
            }
            uint32_t row_scores[1024];
            uint32_t score = 0;
            int best_y = 0;
            int y;
            rotate_angle(bitmap.pixels, rotated, bitmap.width, bitmap.height,
                         s_fallback_angles[quarter]);
            for (y = 0; y < bitmap.height; y++) {
                row_scores[y] = row_score(rotated, bitmap.width, y);
            }
            for (y = 0; y <= bitmap.height - ROI_HEIGHT; y += ROW_STEP) {
                uint32_t band_score = 0;
                int row;
                for (row = y; row < y + ROI_HEIGHT; row++) {
                    band_score += row_scores[row];
                }
                if (band_score > score) {
                    score = band_score;
                    best_y = y;
                }
            }
            candidates[candidate_count].quarter_turns = 0;
            candidates[candidate_count].angle_deg = s_fallback_angles[quarter];
            candidates[candidate_count].fallback = 1;
            candidates[candidate_count].y = best_y;
            candidates[candidate_count].line_y = best_y + ROI_HEIGHT / 2;
            candidates[candidate_count].slope_q16 = 0;
            candidates[candidate_count].score = score;
            candidate_count++;
        }
        qsort(candidates, (size_t)candidate_count, sizeof(candidates[0]), candidate_compare);
        if (fallback_angle_text) {
            for (quarter = 0; quarter < candidate_count; quarter++) {
                if (candidates[quarter].angle_deg == atoi(fallback_angle_text)) {
                    candidates[0] = candidates[quarter];
                    candidate_count = 1;
                    break;
                }
            }
        }
        started = monotonic_us();
        for (quarter = 0; quarter < candidate_count &&
             quarter < MAX_TRIAL_CANDIDATES && !found; quarter++) {
            XcrGrayImage image = {0};
            XcrBarcodeResult result = {0};
            XcrStatus status = XCR_NO_RESULT;
            int result_count = 0;
            candidate_t *candidate = &candidates[quarter];
            XcrDecoder *active_decoder;
            uint8_t *variants[9];
            int variant_count = 2;
            int variant;

            if (!candidate->fallback && quarter >= 8) {
                continue;
            }
            active_decoder = candidate->fallback ? fallback_decoder : decoder;

            if (candidate->angle_deg != 0) {
                rotate_angle(bitmap.pixels, rotated, bitmap.width, bitmap.height,
                             candidate->angle_deg);
            } else {
                rotate_quarter(bitmap.pixels, rotated, bitmap.width, bitmap.height,
                               candidate->quarter_turns);
            }
            build_roi(rotated, bitmap.width, bitmap.height, candidate, roi);
            if (!enhance(roi, enhanced_roi, (size_t)bitmap.width * ROI_HEIGHT)) {
                memcpy(enhanced_roi, roi, (size_t)bitmap.width * ROI_HEIGHT);
            }
            variants[0] = roi;
            variants[1] = enhanced_roi;
            enhance_rows(roi, row_enhanced_roi, bitmap.width, ROI_HEIGHT, 0);
            variants[2] = row_enhanced_roi;
            enhance_rows(roi, binary_roi, bitmap.width, ROI_HEIGHT, 1);
            variants[3] = binary_roi;
            enhance_local(roi, local_roi, bitmap.width, ROI_HEIGHT, 0);
            enhance_local(roi, local_binary_roi, bitmap.width, ROI_HEIGHT, 1);
            enhance_2d(roi, enhanced_2d_roi, horizontal_mean,
                       bitmap.width, ROI_HEIGHT, 16, 4);
            scale_horizontal(roi, scaled_roi, bitmap.width, ROI_HEIGHT);
            scale_horizontal(enhanced_roi, scaled_enhanced_roi,
                             bitmap.width, ROI_HEIGHT);
            variants[4] = local_roi;
            variants[5] = local_binary_roi;
            variants[6] = enhanced_2d_roi;
            variants[7] = scaled_roi;
            variants[8] = scaled_enhanced_roi;
            variant_count = candidate->fallback ? 7 : 9;
            for (variant = 0; variant < variant_count && !found; variant++) {
                image.width = variant >= 7 ? bitmap.width * 2 : bitmap.width;
                image.height = ROI_HEIGHT;
                image.stride = image.width;
                image.pixels = variants[variant];
                memset(&result, 0, sizeof(result));
                result_count = 0;
                status = xcr_decoder_decode(active_decoder, &image, &result, 1,
                                            &result_count);
                if (trace_variants) {
                    printf("TRACE %s q=%d angle=%d y=%d line=%d slope=%d variant=%d status=%s results=%d payload=%s\n",
                           fields[1], candidate->quarter_turns, candidate->angle_deg,
                           candidate->y, candidate->line_y,
                           candidate->slope_q16, variant,
                           xcr_status_name(status), result_count,
                           result_count > 0 ? result.data : "");
                }
                if (expected_match(&result, result_count, fields[2])) {
                    found = 1;
                    if (dump_roi_dir) {
                        char dump_path[PATH_CAPACITY];
                        const char *base = strrchr(fields[1], '/');
                        if (!base) base = strrchr(fields[1], '\\');
                        base = base ? base + 1 : fields[1];
                        if (snprintf(dump_path, sizeof(dump_path), "%s/%s",
                                     dump_roi_dir, base) < (int)sizeof(dump_path)) {
                            char *extension = strrchr(dump_path, '.');
                            if (extension) *extension = '\0';
                            snprintf(dump_path + strlen(dump_path),
                                     sizeof(dump_path) - strlen(dump_path),
                                     "_v%d.bmp", variant);
                            write_gray_bmp(dump_path, variants[variant],
                                           variant >= 7 ? bitmap.width * 2 : bitmap.width,
                                           ROI_HEIGHT);
                        }
                    }
                    printf("OK   %s q=%d angle=%d slope=%d variant=%d status=%s payload=%s\n",
                           fields[1], candidate->quarter_turns, candidate->angle_deg,
                           candidate->slope_q16,
                           variant, xcr_status_name(status), result.data);
                }
            }
        }
        elapsed = monotonic_us() - started;
        total++;
        if (found) passed++; else failed++;
        if (!found) {
            printf("MISS %s elapsed_ms=%.3f candidates=", fields[1],
                   (double)elapsed / 1000.0);
            for (quarter = 0; quarter < candidate_count; quarter++) {
                printf(" q%d:y%d:l%d:s%d:r%lu", candidates[quarter].quarter_turns,
                       candidates[quarter].y, candidates[quarter].line_y,
                       candidates[quarter].slope_q16,
                       (unsigned long)candidates[quarter].score);
            }
            putchar('\n');
        } else {
            printf("     elapsed_ms=%.3f\n", (double)elapsed / 1000.0);
        }
        free(rotated); free(roi); free(enhanced_roi);
        free(row_enhanced_roi); free(binary_roi); free(local_roi);
        free(local_binary_roi); free(enhanced_2d_roi); free(horizontal_mean);
        free(scaled_roi); free(scaled_enhanced_roi);
        image_free(&bitmap);
    }
    fclose(manifest);
    xcr_decoder_destroy(fallback_decoder);
    xcr_decoder_destroy(decoder);
    printf("TOTAL=%d PASSED=%d FAILED=%d RATE=%.2f%%\n", total, passed, failed,
           total ? 100.0 * (double)passed / (double)total : 0.0);
    return failed == 0 ? 0 : 1;
}
