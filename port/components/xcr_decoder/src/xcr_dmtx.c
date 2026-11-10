#include "barcode.h"
#include "datamatrix.h"
#if CONFIG_XCR_DATAMATRIX_LIBDMTX
#include "dmtx_p4.h"
#endif
#include "xcr_port.h"
#include "xcr_dmtx.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif
#if XCR_MARKER_DEBUG
#include <stdio.h>
#endif

#if defined(XCR_DM_TRACE_STAGES) && !defined(ESP_PLATFORM)
#include <stdio.h>
static void trace_marker_pgm(const char *name, const uint8_t *pixels, int width, int height, int bits)
{
    const char *directory = getenv("XCR_DM_TRACE_DIR");
    if (!directory || !*directory) return;
    char path[512];
    if (snprintf(path, sizeof(path), "%s/%s", directory, name) >= (int)sizeof(path)) return;
    FILE *output = fopen(path, "wb");
    if (!output) return;
    fprintf(output, "P5\n%d %d\n255\n", width, height);
    for (int pixel = 0; pixel < width * height; pixel++) {
        uint8_t value = bits ? (pixels[pixel] ? 0 : 255) : pixels[pixel];
        fwrite(&value, 1, 1, output);
    }
    fclose(output);
}

#else
#define trace_marker_pgm(name, pixels, width, height, bits) ((void)0)
#endif

static XcrDmtxStats last_stats;

#ifndef XCR_DM_NORMALIZED_SIDE
#define XCR_DM_NORMALIZED_SIDE 128
#endif

static int marker_step(const Image *image)
{
    (void)image;
    /* The printed external markers are about 20 sensor pixels wide at
     * 1280x720. A 12-pixel step collapses each marker to one or two samples. */
    return 6;
}

typedef struct {
    int x;
    int y;
    int w;
    int h;
    int area;
} MarkerCandidate;

#if defined(XCR_DM_TRACE_STAGES) && !defined(ESP_PLATFORM)
static void trace_marker_boxes(const char *name, const MarkerCandidate *candidates, int count, int step)
{
    const char *directory = getenv("XCR_DM_TRACE_DIR");
    if (!directory || !*directory) return;
    char path[512];
    if (snprintf(path, sizeof(path), "%s/%s", directory, name) >= (int)sizeof(path)) return;
    FILE *output = fopen(path, "w");
    if (!output) return;
    fprintf(output, "x,y,w,h,area\n");
    for (int index = 0; index < count; index++)
        fprintf(output, "%d,%d,%d,%d,%d\n", candidates[index].x * step,
                candidates[index].y * step, candidates[index].w * step,
                candidates[index].h * step, candidates[index].area);
    fclose(output);
}
#else
#define trace_marker_boxes(name, candidates, count, step) ((void)0)
#endif

#if CONFIG_XCR_DATAMATRIX_LIBDMTX
static DmtxRegion *scan_marker_hints(DmtxDecode *decode, int width, int height, int scale)
{
    int scaled_width = width / scale;
    int scaled_height = height / scale;
    int points[32][2];
    int center_x = scaled_width / 2;
    int center_y = scaled_height / 2;
    int point_count = 0;
    if (scaled_width < 12 || scaled_height < 12)
        return NULL;

    for (int margin = 2; margin <= 8; margin++) {
        points[point_count][0] = margin; points[point_count++][1] = center_y;
        points[point_count][0] = scaled_width - 1 - margin; points[point_count++][1] = center_y;
        points[point_count][0] = center_x; points[point_count++][1] = margin;
        points[point_count][0] = center_x; points[point_count++][1] = scaled_height - 1 - margin;
        if (point_count >= 28) break;
    }
    for (int point = 0; point < point_count; point++) {
        DmtxRegion *region = dmtxRegionScanPixel(decode, points[point][0], points[point][1]);
        if (region)
            return region;
    }
    return NULL;
}

static uint8_t *downsample_roi(const uint8_t *pixels, int image_width,
                               int x0, int y0, int width, int height,
                               int factor, int *out_width, int *out_height)
{
    int reduced_width = (width + factor - 1) / factor;
    int reduced_height = (height + factor - 1) / factor;
    uint8_t *reduced = malloc((size_t)reduced_width * reduced_height);
    if (!reduced) return NULL;
    for (int y = 0; y < reduced_height; y++) {
        int source_y = y0 + y * factor + factor / 2;
        if (source_y >= y0 + height) source_y = y0 + height - 1;
        for (int x = 0; x < reduced_width; x++) {
            int source_x = x0 + x * factor + factor / 2;
            if (source_x >= x0 + width) source_x = x0 + width - 1;
            reduced[(size_t)y * reduced_width + x] =
                pixels[(size_t)source_y * image_width + source_x];
        }
    }
    *out_width = reduced_width;
    *out_height = reduced_height;
    return reduced;
}

#endif

static int marker_direction_roi(const MarkerCandidate *origin,
                                const MarkerCandidate *horizontal,
                                const MarkerCandidate *vertical,
                                int step, const Image *image, int roi[4],
                                int *orientation)
{
    int origin_x = origin->x * 2 + origin->w;
    int origin_y = origin->y * 2 + origin->h;
    int horizontal_x = horizontal->x * 2 + horizontal->w;
    int horizontal_y = horizontal->y * 2 + horizontal->h;
    int vertical_x = vertical->x * 2 + vertical->w;
    int vertical_y = vertical->y * 2 + vertical->h;
    int horizontal_dx = horizontal_x - origin_x;
    int horizontal_dy = horizontal_y - origin_y;
    int vertical_dx = vertical_x - origin_x;
    int vertical_dy = vertical_y - origin_y;
    int horizontal_length = abs(horizontal_dx) + abs(horizontal_dy);
    int vertical_length = abs(vertical_dx) + abs(vertical_dy);
    int horizontal_offset = origin->w + horizontal->w;
    int vertical_offset = origin->h + vertical->h;
    int horizontal_inner_x;
    int horizontal_inner_y;
    int vertical_inner_x;
    int vertical_inner_y;
    int points_x[4];
    int points_y[4];
    int minimum_x;
    int maximum_x;
    int minimum_y;
    int maximum_y;

    if (horizontal_length < 1 || vertical_length < 1)
        return 0;

    if (orientation) {
        if (abs(horizontal_dx) >= abs(horizontal_dy))
            *orientation = horizontal_dx < 0 ? 0 : 2;
        else
            *orientation = horizontal_dy < 0 ? 1 : 3;
    }

    /* The marker is 2 modules wide and the gap is 1 module, so the distance
     * from a marker centre to the code inner edge is 2 modules, or one
     * marker width. Coordinates are kept doubled here. */
    horizontal_inner_x = horizontal_dx * horizontal_offset / horizontal_length;
    horizontal_inner_y = horizontal_dy * horizontal_offset / horizontal_length;
    vertical_inner_x = vertical_dx * vertical_offset / vertical_length;
    vertical_inner_y = vertical_dy * vertical_offset / vertical_length;

    points_x[0] = origin_x + horizontal_inner_x + vertical_inner_x;
    points_y[0] = origin_y + horizontal_inner_y + vertical_inner_y;
    points_x[1] = horizontal_x - horizontal_inner_x + vertical_inner_x;
    points_y[1] = horizontal_y - horizontal_inner_y + vertical_inner_y;
    points_x[2] = horizontal_x - horizontal_inner_x + vertical_x - origin_x - vertical_inner_x;
    points_y[2] = horizontal_y - horizontal_inner_y + vertical_y - origin_y - vertical_inner_y;
    points_x[3] = vertical_x + horizontal_inner_x - vertical_inner_x;
    points_y[3] = vertical_y + horizontal_inner_y - vertical_inner_y;

    minimum_x = maximum_x = points_x[0];
    minimum_y = maximum_y = points_y[0];
    for (int point = 1; point < 4; point++) {
        if (points_x[point] < minimum_x) minimum_x = points_x[point];
        if (points_x[point] > maximum_x) maximum_x = points_x[point];
        if (points_y[point] < minimum_y) minimum_y = points_y[point];
        if (points_y[point] > maximum_y) maximum_y = points_y[point];
    }

    /* Keep a generous quiet-zone margin.  The marker spacing is measured on
     * a sparse grid, so a two-sample margin can clip rotated corners after
     * integer rounding. */
    roi[0] = (minimum_x / 2 - 6) * step;
    roi[1] = (minimum_y / 2 - 6) * step;
    roi[2] = (maximum_x / 2 + 7) * step;
    roi[3] = (maximum_y / 2 + 7) * step;
    if (roi[0] < 0) roi[0] = 0;
    if (roi[1] < 0) roi[1] = 0;
    if (roi[2] > image->width) roi[2] = image->width;
    if (roi[3] > image->height) roi[3] = image->height;
    return roi[2] - roi[0] > 20 && roi[3] - roi[1] > 20;
}

static int marker_has_qr_rings(const Image *image, int center_x, int center_y,
                               int width, int height, int threshold)
{
    int votes = 0;
    const int directions[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
    const int radii[3] = {67, 100, 133};
    for (int direction = 0; direction < 8; direction++) {
        int matched = 1;
        for (int ring = 0; ring < 3; ring++) {
            int column = center_x + directions[direction][0] * width * radii[ring] / 100;
            int row = center_y + directions[direction][1] * height * radii[ring] / 100;
            if (column < 0 || row < 0 || column >= image->width || row >= image->height) { matched = 0; break; }
            int dark = image->pixels[row * image->width + column] < threshold;
            if (dark != (ring == 1)) { matched = 0; break; }
        }
        votes += matched;
    }
    return votes >= 6;
}

static int marker_filtered_gray(const Image *image, int sample_x, int sample_y);

static int marker_has_clear_gaps(const Image *image, const float axes[6], int threshold)
{
    const int step = marker_step(image);
    for (int edge = 0; edge < 2; edge++) {
        int values[16];
        int peak = 0;
        for (int sample = 0; sample < 16; sample++) {
            float along = 0.15f + sample * (0.7f / 15);
            int column = (int)lroundf(axes[0] - step * 0.5f + along * axes[2 + edge * 2]);
            int row = (int)lroundf(axes[1] - step * 0.5f + along * axes[3 + edge * 2]);
            values[sample] = column >= 0 && row >= 0 && column < image->width && row < image->height
                ? marker_filtered_gray(image, column, row) : 255;
            if (values[sample] > peak) peak = values[sample];
        }
        int local_threshold = peak * 65 / 100;
        if (local_threshold > threshold) local_threshold = threshold;
        int dark = 0;
        for (int sample = 0; sample < 16; sample++) dark += values[sample] < local_threshold;
        if (dark > 2) return 0;
    }
    return 1;
}

static int marker_has_solid_edges(const Image *image, const float axes[6], int threshold)
{
    const int step = marker_step(image);
    for (int edge = 0; edge < 2; edge++) {
        int best = 0;
        for (int band = -3; band <= 3; band++) {
            float fixed = 11.5f / 14 + band * 0.025f;
            int dark = 0;
            for (int sample = 0; sample < 16; sample++) {
                float along = (2.5f + sample * (9.0f / 15)) / 14;
                float horizontal = edge ? along : fixed;
                float vertical = edge ? fixed : along;
                int column = (int)lroundf(axes[0] - step * 0.5f + horizontal * axes[2] + vertical * axes[4]);
                int row = (int)lroundf(axes[1] - step * 0.5f + horizontal * axes[3] + vertical * axes[5]);
                if (column >= 0 && row >= 0 && column < image->width && row < image->height)
                    dark += image->pixels[row * image->width + column] < threshold;
            }
            if (dark > best) best = dark;
            if (best >= 14) break;
        }
        if (best < 14) return 0;
    }
    return 1;
}

static int marker_filtered_gray(const Image *image, int sample_x, int sample_y)
{
    const int stride = image->width;
    const uint8_t *pixels = image->pixels + sample_y * stride + sample_x;
    if (sample_x < 1 || sample_y < 1 || sample_x >= stride - 1 || sample_y >= image->height - 1)
        return *pixels;
    return (pixels[-stride - 1] + pixels[-stride] + pixels[-stride + 1] +
            pixels[-1] + pixels[0] + pixels[1] +
            pixels[stride - 1] + pixels[stride] + pixels[stride + 1] + 4) / 9;
}

static int find_marker_roi_slice(const Image *image, int sample_left, int sample_right,
                                 int roi[4], int *orientation, float axes[6],
                                 int *score_out, unsigned *qr_rejected)
{
    const int step = marker_step(image);
    const int width = sample_right - sample_left;
    const int height = (image->height + step - 1) / step;
    if (width <= 0) return 0;
    const size_t count = (size_t)width * (size_t)height;
    XcrMemClass queue_class = XCR_MEM_LARGE;
#ifdef ESP_PLATFORM
    if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) > count * (sizeof(int) + 1) + 16384)
        queue_class = XCR_MEM_FAST;
#endif
    int *queue = xcr_alloc(count * (sizeof(*queue) + 1), queue_class);
    uint8_t *mask = queue ? (uint8_t *)(queue + count) : NULL;
    MarkerCandidate candidates[32];
    int candidate_count = 0;
    int minimum = 255;
    int maximum = 0;
    if (!mask || !queue) {
        xcr_free(queue);
        return 0;
    }
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int source_x = (x + sample_left) * step;
            int value = image->pixels[(size_t)y * step * image->width + source_x];
            if (value < 48) value = marker_filtered_gray(image, source_x, y * step);
            mask[y * width + x] = value;
            queue[y * width + x] = value + (x ? queue[y * width + x - 1] : 0) +
                (y ? queue[(y - 1) * width + x] : 0) -
                (x && y ? queue[(y - 1) * width + x - 1] : 0);
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
        }
    }
    trace_marker_pgm("02_marker_sample_6x.pgm", mask, width, height, 0);
    int threshold = minimum + (maximum - minimum) * 35 / 100;
    if (threshold < 24) threshold = 24;
    if (threshold > 160) threshold = 160;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int left = x > 6 ? x - 6 : 0;
            int top = y > 6 ? y - 6 : 0;
            int right = x + 6 < width ? x + 6 : width - 1;
            int bottom = y + 6 < height ? y + 6 : height - 1;
            int sum = queue[bottom * width + right] - (left ? queue[bottom * width + left - 1] : 0) -
                (top ? queue[(top - 1) * width + right] : 0) +
                (left && top ? queue[(top - 1) * width + left - 1] : 0);
            int area = (right - left + 1) * (bottom - top + 1);
            int value = mask[y * width + x];
            mask[y * width + x] = value < threshold && value * area * 100 < sum * 75;
        }
    }
    trace_marker_pgm("03_marker_binary.pgm", mask, width, height, 1);
    for (int start_y = 0; start_y < height; start_y++) {
        for (int start_x = 0; start_x < width; start_x++) {
            size_t start = (size_t)start_y * width + start_x;
            if (!mask[start]) continue;
            int head = 0;
            int tail = 0;
            int min_x = start_x, max_x = start_x, min_y = start_y, max_y = start_y;
            mask[start] = 0;
            queue[tail++] = start_y * width + start_x;
            while (head < tail) {
                int point = queue[head++];
                int x = point % width;
                int y = point / width;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                const int delta_x[4] = {-1, 1, 0, 0};
                const int delta_y[4] = {0, 0, -1, 1};
                for (int direction = 0; direction < 4; direction++) {
                    int next_x = x + delta_x[direction];
                    int next_y = y + delta_y[direction];
                    if (next_x < 0 || next_x >= width || next_y < 0 || next_y >= height) continue;
                    int next = next_y * width + next_x;
                    if (!mask[next]) continue;
                    mask[next] = 0;
                    queue[tail++] = next;
                }
            }
            int component_width = max_x - min_x + 1;
            int component_height = max_y - min_y + 1;
            int component_area = tail;
            if (component_width >= 2 && component_height >= 2 && component_width <= 32 && component_height <= 32) {
                int filtered_min_x = width, filtered_max_x = -1;
                int filtered_min_y = height, filtered_max_y = -1;
                int filtered_area = 0;
                for (int sample = 0; sample < tail; sample++) {
                    int sample_x = queue[sample] % width;
                    int sample_y = queue[sample] / width;
                    if (marker_filtered_gray(image, (sample_x + sample_left) * step, sample_y * step) >= threshold) continue;
                    if (sample_x < filtered_min_x) filtered_min_x = sample_x;
                    if (sample_x > filtered_max_x) filtered_max_x = sample_x;
                    if (sample_y < filtered_min_y) filtered_min_y = sample_y;
                    if (sample_y > filtered_max_y) filtered_max_y = sample_y;
                    filtered_area++;
                }
                if (!filtered_area) continue;
                min_x = filtered_min_x;
                max_x = filtered_max_x;
                min_y = filtered_min_y;
                max_y = filtered_max_y;
                component_width = max_x - min_x + 1;
                component_height = max_y - min_y + 1;
                component_area = filtered_area;
            }
            if (component_width < 2 || component_height < 2 || component_width > 32 || component_height > 32)
                continue;
            if (component_area * 10 < component_width * component_height * 4)
                continue;
            if (component_width * 10 < component_height * 6 || component_height * 10 < component_width * 6)
                continue;
            int center_x = (min_x + max_x + 2 * sample_left) * step / 2;
            int center_y = (min_y + max_y) * step / 2;
            int dark_sum = 0;
            int dark_max = 0;
            int light_sum = 0;
            int light_count = 0;
            for (int offset_y = -1; offset_y <= 1; offset_y++) {
                for (int offset_x = -1; offset_x <= 1; offset_x++) {
                    int sample_x = center_x + offset_x * component_width * step / 6;
                    int sample_y = center_y + offset_y * component_height * step / 6;
                    int value = marker_filtered_gray(image, sample_x, sample_y);
                    dark_sum += value;
                    if (value > dark_max) dark_max = value;
                }
            }
            for (int side = 0; side < 4; side++) {
                for (int offset = -1; offset <= 1; offset++) {
                    int sample_x = center_x + (side < 2 ? (side ? 1 : -1) * (component_width + 2) * step / 2 : offset * component_width * step / 4);
                    int sample_y = center_y + (side >= 2 ? (side == 3 ? 1 : -1) * (component_height + 2) * step / 2 : offset * component_height * step / 4);
                    if (sample_x < 0 || sample_x >= image->width || sample_y < 0 || sample_y >= image->height) continue;
                    light_sum += marker_filtered_gray(image, sample_x, sample_y);
                    light_count++;
                }
            }
            if (light_count < 8) continue;
            int contrast_floor = light_sum / light_count / 4;
            if (contrast_floor < 3) contrast_floor = 3;
            if (contrast_floor > 20) contrast_floor = 20;
            if (light_sum * 9 < (dark_sum + 9 * contrast_floor) * light_count) continue;
            if (dark_max * light_count * 100 >= light_sum * 75) continue;
            if (marker_has_qr_rings(image, center_x, center_y, component_width * step,
                                    component_height * step, (dark_sum / 9 + light_sum / light_count) / 2)) {
                if (qr_rejected) (*qr_rejected)++;
                continue;
            }
            if (candidate_count < (int)(sizeof(candidates) / sizeof(candidates[0])))
                candidates[candidate_count++] = (MarkerCandidate){min_x + sample_left, min_y, component_width, component_height, component_area};
        }
    }
    int found = 0;
    int best_score = 0;
    trace_marker_boxes("04_marker_candidates.csv", candidates, candidate_count, step);
#if XCR_MARKER_DEBUG
    for (int candidate = 0; candidate < candidate_count; candidate++)
        fprintf(stderr, "marker %d: %d,%d %dx%d area=%d\n", candidate, candidates[candidate].x,
                candidates[candidate].y, candidates[candidate].w, candidates[candidate].h, candidates[candidate].area);
#endif
    for (int first = 0; first < candidate_count; first++) {
        for (int second = first + 1; second < candidate_count; second++) {
            for (int third = second + 1; third < candidate_count; third++) {
                MarkerCandidate *a = &candidates[first];
                MarkerCandidate *b = &candidates[second];
                MarkerCandidate *c = &candidates[third];
                int min_x = a->x < b->x ? a->x : b->x;
                if (c->x < min_x) min_x = c->x;
                int max_x = a->x > b->x ? a->x : b->x;
                if (c->x > max_x) max_x = c->x;
                int min_y = a->y < b->y ? a->y : b->y;
                if (c->y < min_y) min_y = c->y;
                int max_y = a->y > b->y ? a->y : b->y;
                if (c->y > max_y) max_y = c->y;
                int span_x = max_x - min_x;
                int span_y = max_y - min_y;
                if (span_x < 12 || span_y < 12)
                    continue;
                int minimum_width = a->w < b->w ? a->w : b->w;
                if (c->w < minimum_width) minimum_width = c->w;
                int maximum_width = a->w > b->w ? a->w : b->w;
                if (c->w > maximum_width) maximum_width = c->w;
                int minimum_height = a->h < b->h ? a->h : b->h;
                if (c->h < minimum_height) minimum_height = c->h;
                int maximum_height = a->h > b->h ? a->h : b->h;
                if (c->h > maximum_height) maximum_height = c->h;
                if (maximum_width > minimum_width * 2 || maximum_height > minimum_height * 2)
                    continue;
                MarkerCandidate *points[3] = {a, b, c};
                int right_angle = 0;
                int geometry_quality = 0;
                int fill_quality = 0;
                int direction_roi[4] = {0};
                float candidate_axes[6] = {0};
                for (int corner = 0; corner < 3; corner++) {
                    MarkerCandidate *origin = points[corner];
                    MarkerCandidate *horizontal = points[(corner + 1) % 3];
                    MarkerCandidate *vertical = points[(corner + 2) % 3];
                    int origin_center_x = origin->x * 2 + origin->w;
                    int origin_center_y = origin->y * 2 + origin->h;
                    int horizontal_center_x = horizontal->x * 2 + horizontal->w;
                    int horizontal_center_y = horizontal->y * 2 + horizontal->h;
                    int vertical_center_x = vertical->x * 2 + vertical->w;
                    int vertical_center_y = vertical->y * 2 + vertical->h;
                    int horizontal_delta_x = horizontal_center_x - origin_center_x;
                    int horizontal_delta_y = horizontal_center_y - origin_center_y;
                    int vertical_delta_x = vertical_center_x - origin_center_x;
                    int vertical_delta_y = vertical_center_y - origin_center_y;
                    int dot = horizontal_delta_x * vertical_delta_x +
                              horizontal_delta_y * vertical_delta_y;
                    int horizontal_length_squared = horizontal_delta_x * horizontal_delta_x +
                                                    horizontal_delta_y * horizontal_delta_y;
                    int vertical_length_squared = vertical_delta_x * vertical_delta_x +
                                                  vertical_delta_y * vertical_delta_y;
                    int marker_extent_sum = origin->w + origin->h + horizontal->w + horizontal->h + vertical->w + vertical->h;
                    int minimum_leg = marker_extent_sum * 4 / 3;
                    int maximum_leg = marker_extent_sum * 9 / 3 + 2;
                    int envelope = abs(horizontal_delta_x) + abs(horizontal_delta_y);
                    int solid_markers = 1;
                    for (int marker = 0; marker < 3; marker++) {
                        MarkerCandidate *candidate = points[marker];
                        if ((int64_t)candidate->area * 100 * envelope * envelope <
                            (int64_t)candidate->w * candidate->h * 70 * horizontal_length_squared)
                            solid_markers = 0;
                    }
                    if (!solid_markers) continue;
                    if (horizontal_length_squared < minimum_leg * minimum_leg || horizontal_length_squared > maximum_leg * maximum_leg ||
                        vertical_length_squared < minimum_leg * minimum_leg || vertical_length_squared > maximum_leg * maximum_leg ||
                        horizontal_length_squared * 3 < vertical_length_squared || vertical_length_squared * 3 < horizontal_length_squared)
                        continue;
                    if (abs(dot) * 5 <= horizontal_length_squared + vertical_length_squared &&
                        horizontal_length_squared > 16 && vertical_length_squared > 16) {
                        int candidate_orientation = 0;
                        if (marker_direction_roi(origin, horizontal, vertical, step, image,
                                                  direction_roi, &candidate_orientation)) {
                            int shorter_leg = horizontal_length_squared < vertical_length_squared ? horizontal_length_squared : vertical_length_squared;
                            int longer_leg = horizontal_length_squared > vertical_length_squared ? horizontal_length_squared : vertical_length_squared;
                            int length_sum = horizontal_length_squared + vertical_length_squared;
                            int quality = (int)((int64_t)1024 * shorter_leg * (length_sum - 4 * abs(dot)) / longer_leg / length_sum);
                            if (quality <= geometry_quality) continue;
                            right_angle = 1;
                            geometry_quality = quality;
                            fill_quality = 1024;
                            for (int marker = 0; marker < 3; marker++) {
                                MarkerCandidate *candidate = points[marker];
                                int fill = (int)((int64_t)candidate->area * 100 * envelope * envelope /
                                    ((int64_t)candidate->w * candidate->h * horizontal_length_squared));
                                if (fill > 100) fill = 100;
                                fill_quality = fill_quality * fill / 100;
                            }
                            candidate_axes[0] = origin_center_x * step * 0.5f;
                            candidate_axes[1] = origin_center_y * step * 0.5f;
                            int cross = horizontal_delta_x * vertical_delta_y - horizontal_delta_y * vertical_delta_x;
                            candidate_axes[2] = (cross > 0 ? horizontal_delta_x : vertical_delta_x) * step * 0.5f;
                            candidate_axes[3] = (cross > 0 ? horizontal_delta_y : vertical_delta_y) * step * 0.5f;
                            candidate_axes[4] = (cross > 0 ? vertical_delta_x : horizontal_delta_x) * step * 0.5f;
                            candidate_axes[5] = (cross > 0 ? vertical_delta_y : horizontal_delta_y) * step * 0.5f;
                            if (orientation) *orientation = candidate_orientation;
                        }
                    }
                }
                if (!right_angle) continue;
                int score = geometry_quality;
                int minimum_area = a->area < b->area ? a->area : b->area;
                if (c->area < minimum_area) minimum_area = c->area;
                int maximum_area = a->area > b->area ? a->area : b->area;
                if (c->area > maximum_area) maximum_area = c->area;
                score = score * minimum_area / maximum_area;
                score = score * fill_quality / 1024;
                score *= minimum_area < 9 ? minimum_area : 9;
#if XCR_MARKER_DEBUG
                fprintf(stderr, "triple %d,%d,%d score=%d quality=%d\n", first, second, third, score, geometry_quality);
#endif
                if (score <= best_score) continue;
                if (!marker_has_clear_gaps(image, candidate_axes, threshold)) continue;
                if (!marker_has_solid_edges(image, candidate_axes, threshold)) continue;
                best_score = score;
                MarkerCandidate chosen[3] = {*a, *b, *c};
                trace_marker_boxes("04_selected_markers.csv", chosen, 3, step);
                if (axes) memcpy(axes, candidate_axes, sizeof(candidate_axes));
                memcpy(roi, direction_roi, sizeof(direction_roi));
                found = roi[2] - roi[0] > 20 && roi[3] - roi[1] > 20;
            }
        }
    }
    xcr_free(queue);
    if (score_out) *score_out = best_score;
    return found;
}

static int find_marker_roi(const Image *image, int roi[4], int *orientation, float axes[6])
{
    return find_marker_roi_slice(image, 0, (image->width + marker_step(image) - 1) / marker_step(image),
                                 roi, orientation, axes, NULL, &last_stats.qr_rejected);
}

#ifdef ESP_PLATFORM
typedef struct {
    Image image;
    int sample_left;
    int sample_right;
    int roi[4];
    int orientation;
    float axes[6];
    int score;
    unsigned qr_rejected;
    int found;
    uint64_t runtime_us;
} MarkerSliceJob;

static MarkerSliceJob s_right_job;
static TaskHandle_t s_marker_worker;
static SemaphoreHandle_t s_marker_done;

static void marker_slice_worker(void *argument)
{
    (void)argument;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t started = xcr_port_time_us();
        s_right_job.found = find_marker_roi_slice(&s_right_job.image,
            s_right_job.sample_left, s_right_job.sample_right, s_right_job.roi,
            &s_right_job.orientation, s_right_job.axes, &s_right_job.score,
            &s_right_job.qr_rejected);
        s_right_job.runtime_us = xcr_port_time_us() - started;
        xSemaphoreGive(s_marker_done);
    }
}

int xcr_dmtx_parallel_start(void)
{
    if (s_marker_worker) return 1;
    s_marker_done = xSemaphoreCreateBinary();
    if (!s_marker_done) return 0;
    if (xTaskCreatePinnedToCore(marker_slice_worker, "dm_marker_right", 8192,
                                NULL, 7, &s_marker_worker, 0) != pdPASS) {
        vSemaphoreDelete(s_marker_done);
        s_marker_done = NULL;
        return 0;
    }
    return 1;
}

static int find_marker_roi_parallel(const Image *image, int roi[4], int *orientation, float axes[6])
{
    int sampled_width = (image->width + marker_step(image) - 1) / marker_step(image);
    if (!s_marker_worker || sampled_width < 100) return find_marker_roi(image, roi, orientation, axes);
    /* Forty sampled columns (240 sensor pixels) cover the usual marker
     * spread across the centre. A full-frame search handles wider symbols. */
    int centre = sampled_width / 2;
    int overlap = 40;
    MarkerSliceJob left = {.image = *image, .sample_left = 0,
                           .sample_right = centre + overlap};
    memset(&s_right_job, 0, sizeof(s_right_job));
    s_right_job.image = *image;
    s_right_job.sample_left = centre - overlap;
    s_right_job.sample_right = sampled_width;
    xSemaphoreTake(s_marker_done, 0);
    xTaskNotifyGive(s_marker_worker);
    uint64_t started = xcr_port_time_us();
    left.found = find_marker_roi_slice(&left.image, left.sample_left, left.sample_right,
        left.roi, &left.orientation, left.axes, &left.score, &left.qr_rejected);
    left.runtime_us = xcr_port_time_us() - started;
    /* The capture buffer must stay owned by the decoder until Core 0 stops
     * reading it, even if this frame has passed its validity deadline. */
    xSemaphoreTake(s_marker_done, portMAX_DELAY);
    last_stats.marker_parallel_used = 1;
    last_stats.marker_core0_us = s_right_job.runtime_us;
    last_stats.marker_core1_us = left.runtime_us;
    last_stats.qr_rejected += left.qr_rejected + s_right_job.qr_rejected;
    MarkerSliceJob *winner = NULL;
    if (left.found) winner = &left;
    if (s_right_job.found && (!winner || s_right_job.score > winner->score))
        winner = &s_right_job;
    if (winner) {
        memcpy(roi, winner->roi, sizeof(winner->roi));
        if (orientation) *orientation = winner->orientation;
        if (axes) memcpy(axes, winner->axes, sizeof(winner->axes));
        return 1;
    }
    last_stats.marker_full_fallback = 1;
    return find_marker_roi(image, roi, orientation, axes);
}
#elif defined(XCR_DM_SPLIT_TEST)
int xcr_dmtx_parallel_start(void) { return 1; }
static int find_marker_roi_parallel(const Image *image, int roi[4], int *orientation, float axes[6])
{
    const int width = (image->width + marker_step(image) - 1) / marker_step(image);
    const int centre = width / 2;
    const int overlap = 40;
    int left_roi[4] = {0}, right_roi[4] = {0};
    int left_orientation = 0, right_orientation = 0;
    int left_score = 0, right_score = 0;
    float left_axes[6] = {0}, right_axes[6] = {0};
    unsigned left_rejected = 0, right_rejected = 0;
    int left = find_marker_roi_slice(image, 0, centre + overlap, left_roi,
        &left_orientation, left_axes, &left_score, &left_rejected);
    int right = find_marker_roi_slice(image, centre - overlap, width, right_roi,
        &right_orientation, right_axes, &right_score, &right_rejected);
    last_stats.marker_parallel_used = 1;
    last_stats.qr_rejected += left_rejected + right_rejected;
    if (left && (!right || left_score >= right_score)) {
        memcpy(roi, left_roi, sizeof(left_roi));
        *orientation = left_orientation;
        memcpy(axes, left_axes, sizeof(left_axes));
        return 1;
    }
    if (right) {
        memcpy(roi, right_roi, sizeof(right_roi));
        *orientation = right_orientation;
        memcpy(axes, right_axes, sizeof(right_axes));
        return 1;
    }
    last_stats.marker_full_fallback = 1;
    return find_marker_roi(image, roi, orientation, axes);
}
#else
int xcr_dmtx_parallel_start(void) { return 0; }
#define find_marker_roi_parallel find_marker_roi
#endif

static void normalize_marker_roi(const Image *image, const int roi[4], int orientation,
                                 uint8_t *normalized)
{
    int width = roi[2] - roi[0];
    int height = roi[3] - roi[1];
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int source_x;
            int source_y;
            if (orientation == 1) {
                source_x = width - 1 - y;
                source_y = x;
            } else if (orientation == 2) {
                source_x = width - 1 - x;
                source_y = height - 1 - y;
            } else if (orientation == 3) {
                source_x = y;
                source_y = height - 1 - x;
            } else {
                source_x = x;
                source_y = y;
            }
            normalized[(size_t)y * width + x] =
                image->pixels[(size_t)(roi[1] + source_y) * image->width +
                              roi[0] + source_x];
        }
    }
}

static void map_normalized_point(int normalized_x, int normalized_y, int width, int height,
                                 const int roi[4], int orientation, int *x, int *y)
{
    int source_x;
    int source_y;
    if (orientation == 1) {
        source_x = width - 1 - normalized_y;
        source_y = normalized_x;
    } else if (orientation == 2) {
        source_x = width - 1 - normalized_x;
        source_y = height - 1 - normalized_y;
    } else if (orientation == 3) {
        source_x = normalized_y;
        source_y = height - 1 - normalized_x;
    } else {
        source_x = normalized_x;
        source_y = normalized_y;
    }
    *x = roi[0] + source_x;
    *y = roi[1] + source_y;
}

void xcr_dmtx_stats_get(XcrDmtxStats *out)
{
    *out = last_stats;
}

#if CONFIG_XCR_DATAMATRIX_LIBDMTX
int xcr_dmtx_decode(const Image *image, BarcodeResult *result, const XcrDeadline *deadline)
{
    uint64_t begin = xcr_port_time_us();
    memset(&last_stats, 0, sizeof(last_stats));
    uint64_t prescreen_begin = xcr_port_time_us();
    int marker_roi[4] = {0};
    int marker_orientation = 0;
    int marker_roi_valid = find_marker_roi(image, marker_roi, &marker_orientation, NULL);
    last_stats.prescreen_us = xcr_port_time_us() - prescreen_begin;
    last_stats.marker_roi_valid = marker_roi_valid;
    if (marker_roi_valid) memcpy(last_stats.marker_roi, marker_roi, sizeof(marker_roi));
    /* This project uses the three external markers as the fast presence
     * detector.  Without them the frame is not a supported symbol, so do
     * not fall through into an expensive full-frame libdmtx search. */
    if (!marker_roi_valid) {
        last_stats.total_us = xcr_port_time_us() - begin;
        return 0;
    }
    DmtxImage *input = dmtxImageCreate(image->pixels, image->width, image->height, DmtxPack8bppK);
    if (!input) return 0;
    uint8_t *normalized_pixels = NULL;
    DmtxImage *normalized_input = NULL;
    int normalized_width = 0;
    int normalized_height = 0;
    if (marker_roi_valid) {
        normalized_width = marker_roi[2] - marker_roi[0];
        normalized_height = marker_roi[3] - marker_roi[1];
        normalized_pixels = malloc((size_t)normalized_width * normalized_height);
        if (normalized_pixels) {
            normalize_marker_roi(image, marker_roi, marker_orientation, normalized_pixels);
            normalized_input = dmtxImageCreate(normalized_pixels, normalized_width,
                                               normalized_height, DmtxPack8bppK);
            if (!normalized_input) {
                free(normalized_pixels);
                normalized_pixels = NULL;
            }
        }
    }
    /* The generated 10x10 symbol has sufficient module width at 8x.  A 4x
     * retry only adds a long failure tail and cannot recover a bad marker ROI
     * because the marker gate has already accepted the frame. */
    const int scales[] = {8};
    for (unsigned attempt = 0; attempt < sizeof(scales) / sizeof(scales[0]); attempt++) {
        if (xcr_deadline_expired(deadline)) break;
        int scale = scales[attempt];
        /* The marker path constrains the symbol to the generated 10x10
         * format.  Going below 4x after that path has failed only repeats a
         * very expensive full-resolution search and creates multi-hundred-ms
         * tails; the next frame is a better sample for the moving scanner. */
        if (marker_roi_valid && scale < 4) break;
        if (image->width / scale < 20 || image->height / scale < 20) continue;
        /* A full-frame decoder owns a downscaled cache, so overlapping tiles
         * only repeat the scan and inflate the long tail.  Keep one pass per
         * scale; the marker ROI pass remains the first, narrower pass when it
         * is available. */
        int tile_size = image->width;
        int tile_step = image->width;
        int pass_count = normalized_input ? 3 : 2;
        for (int pass = 0; pass < pass_count && !result->length; pass++) {
            int full_frame_pass = pass == 2;
            int marker_pass = !full_frame_pass;
            int normalized_pass = normalized_input && pass == 0;
            DmtxImage *pass_input = normalized_pass ? normalized_input : input;
            int source_width = normalized_pass ? normalized_width : image->width;
            int source_height = normalized_pass ? normalized_height : image->height;
            int source_x = normalized_pass ? 0 : (full_frame_pass ? 0 : marker_roi[0]);
            int source_y = normalized_pass ? 0 : (full_frame_pass ? 0 : marker_roi[1]);
            int source_roi_width = normalized_pass ? normalized_width : (full_frame_pass ? image->width : marker_roi[2] - marker_roi[0]);
            int source_roi_height = normalized_pass ? normalized_height : (full_frame_pass ? image->height : marker_roi[3] - marker_roi[1]);
            int decode_scale = scale;
            int coordinate_scale = scale;
            int pass_width = source_width;
            int pass_height = source_height;
            int pass_x = normalized_pass ? 0 : (full_frame_pass ? 0 : marker_roi[0]);
            int pass_y = normalized_pass ? 0 : (full_frame_pass ? 0 : marker_roi[1]);
            int pass_right = source_width;
            int pass_bottom = source_height;
            uint8_t *reduced_pixels = NULL;
            DmtxImage *reduced_input = NULL;
            if (marker_pass && scale == 8) {
                const uint8_t *source_pixels = normalized_pass ? normalized_pixels : image->pixels;
                reduced_pixels = downsample_roi(source_pixels, source_width, source_x, source_y,
                                                source_roi_width, source_roi_height, 8,
                                                &pass_width, &pass_height);
                if (reduced_pixels) {
                    reduced_input = dmtxImageCreate(reduced_pixels, pass_width, pass_height, DmtxPack8bppK);
                    if (reduced_input) {
                        pass_input = reduced_input;
                        decode_scale = 1;
                        coordinate_scale = 8;
                        pass_x = 0;
                        pass_y = 0;
                        pass_right = pass_width;
                        pass_bottom = pass_height;
                    } else {
                        free(reduced_pixels);
                        reduced_pixels = NULL;
                    }
                }
            }
            int pass_step = marker_pass ? image->width : tile_step;
            int pass_tile_size = marker_pass ? (pass_right - pass_x) : tile_size;
        for (int tile_y = pass_y; tile_y < pass_bottom && !xcr_deadline_expired(deadline); tile_y += pass_step) {
            for (int tile_x = pass_x; tile_x < pass_right && !xcr_deadline_expired(deadline); tile_x += pass_step) {
                uint64_t setup = xcr_port_time_us();
                DmtxDecode *decode = dmtxDecodeCreate(pass_input, decode_scale);
                last_stats.setup_us += xcr_port_time_us() - setup;
                if (!decode) break;
                int tile_right = tile_x + pass_tile_size;
                int tile_bottom = tile_y + pass_tile_size;
                if (tile_right > pass_width) tile_right = pass_width;
                if (tile_bottom > pass_height) tile_bottom = pass_height;
                dmtxDecodeSetProp(decode, DmtxPropXmin, tile_x);
                dmtxDecodeSetProp(decode, DmtxPropXmax, tile_right - 1);
                dmtxDecodeSetProp(decode, DmtxPropYmin, tile_y);
                dmtxDecodeSetProp(decode, DmtxPropYmax, tile_bottom - 1);
                if (scale >= 8) dmtxDecodeSetProp(decode, DmtxPropScanGap, marker_pass ? 48 : 32);
                else if (scale >= 4) dmtxDecodeSetProp(decode, DmtxPropScanGap, 32);
                if (marker_roi_valid && marker_pass)
                    dmtxDecodeSetProp(decode, DmtxPropSymbolSize, DmtxSymbol10x10);
                last_stats.scale = scale;
                uint64_t now = xcr_port_time_us();
                uint64_t limit = deadline && deadline->deadline_us ? deadline->deadline_us : begin + 100000;
                uint64_t remaining = limit > now ? limit - now : 0;
                unsigned budget = remaining / 1000;
                if (budget > 22 && attempt < 2) budget = 22;
                DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), budget);
                DmtxRegion *hint_region = marker_pass ? scan_marker_hints(decode, pass_width, pass_height, decode_scale) : NULL;
                if (hint_region)
                    last_stats.hint_hits++;
                while (!xcr_deadline_expired(deadline) && !dmtxTimeExceeded(timeout)) {
            uint64_t find_begin = xcr_port_time_us();
            DmtxRegion *region = hint_region;
            hint_region = NULL;
            if (!region)
                region = dmtxRegionFindNext(decode, &timeout);
            last_stats.find_us += xcr_port_time_us() - find_begin;
            if (!region) break;
            last_stats.candidates++;
            uint64_t decode_begin = xcr_port_time_us();
            DmtxMessage *message = dmtxDecodeMatrixRegion(decode, region, DmtxUndefined);
            last_stats.decode_us += xcr_port_time_us() - decode_begin;
            if (message && message->outputIdx > 0 && message->outputIdx < BARCODE_MAX_LEN) {
                last_stats.decoded_messages++;
                memcpy(result->data, message->output, message->outputIdx);
                result->length = message->outputIdx;
                result->data[result->length] = 0;
                result->symbology = BARCODE_SYM_DATAMATRIX;
                result->confidence = 100;
                DmtxVector2 fitted[4] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};
                result->geometry_valid = 1;
                for (int point = 0; point < 4; point++) {
                    DmtxVector2 raw;
                    if (dmtxMatrix3VMultiply(&raw, &fitted[point], region->fit2raw) != DmtxPass) {
                        result->geometry_valid = 0;
                        break;
                    }
                    int normalized_x = (int)lround(raw.X * coordinate_scale);
                    int normalized_y = source_height - 1 - (int)lround(raw.Y * coordinate_scale);
                    if (normalized_pass) {
                        map_normalized_point(normalized_x, normalized_y, source_width, source_height,
                                             marker_roi, marker_orientation,
                                             &result->corners[point * 2],
                                             &result->corners[point * 2 + 1]);
                    } else {
                        result->corners[point * 2] = pass_x + normalized_x;
                        result->corners[point * 2 + 1] = pass_y + normalized_y;
                    }
                    if (result->corners[point * 2] < 0 || result->corners[point * 2] >= image->width ||
                        result->corners[point * 2 + 1] < 0 || result->corners[point * 2 + 1] >= image->height)
                        result->geometry_valid = 0;
                }
                if (!result->geometry_valid) {
                    int left = pass_x + (int)lround(region->boundMin.X * coordinate_scale);
                    int right = pass_x + (int)lround(region->boundMax.X * coordinate_scale);
                    int top = pass_y + source_height - 1 - (int)lround(region->boundMax.Y * coordinate_scale);
                    int bottom = pass_y + source_height - 1 - (int)lround(region->boundMin.Y * coordinate_scale);
                    if (left < 0) left = 0;
                    if (right >= image->width) right = image->width - 1;
                    if (top < 0) top = 0;
                    if (bottom >= image->height) bottom = image->height - 1;
                    if (right > left && bottom > top) {
                        result->corners[0] = left; result->corners[1] = top;
                        result->corners[2] = right; result->corners[3] = top;
                        result->corners[4] = right; result->corners[5] = bottom;
                        result->corners[6] = left; result->corners[7] = bottom;
                        result->geometry_valid = 1;
                        last_stats.geometry_fallbacks++;
                    }
                }
                if (xcr_deadline_expired(deadline)) result->length = 0;
            }
            if (message) dmtxMessageDestroy(&message);
            dmtxRegionDestroy(&region);
                    if (result->length) break;
                }
                dmtxDecodeDestroy(&decode);
                if (result->length) break;
            }
            if (result->length) break;
        }
        if (reduced_input) dmtxImageDestroy(&reduced_input);
        free(reduced_pixels);
        }
        if (result->length) break;
    }
    if (normalized_input) dmtxImageDestroy(&normalized_input);
    free(normalized_pixels);
    dmtxImageDestroy(&input);
    last_stats.total_us = xcr_port_time_us() - begin;
    return result->length;
}
#endif

typedef struct {
    float along;
    float offset;
} DmEdgeSample;

static inline float dm_gray_unchecked(const Image *image, float horizontal, float vertical)
{
    int column = (int)horizontal, row = (int)vertical;
    float fraction_x = horizontal - column, fraction_y = vertical - row;
    const uint8_t *source = image->pixels + row * image->width + column;
    float upper = source[0] + fraction_x * (source[1] - source[0]);
    float lower = source[image->width] + fraction_x * (source[image->width + 1] - source[image->width]);
    return upper + fraction_y * (lower - upper);
}

static float dm_gray_at(const Image *image, float horizontal, float vertical)
{
    if (horizontal < 0 || vertical < 0 || horizontal >= image->width - 1 || vertical >= image->height - 1)
        return -1;
    return dm_gray_unchecked(image, horizontal, vertical);
}

static int dm_fit_edge(const DmEdgeSample *samples, int count, float *intercept, float *slope)
{
    int best_count = 0;
    float best_error = INFINITY;
    for (int first = 0; first < count; first += 2) {
        for (int second = first + 1; second < count; second += 2) {
            float separation = samples[second].along - samples[first].along;
            if (separation < 0.4f) continue;
            float trial_slope = (samples[second].offset - samples[first].offset) / separation;
            float trial_intercept = samples[first].offset - trial_slope * samples[first].along;
            int inliers = 0;
            float error = 0;
            for (int index = 0; index < count; index++) {
                float residual = fabsf(samples[index].offset - trial_intercept - trial_slope * samples[index].along);
                if (residual < 1.5f) { inliers++; error += residual; }
                else {
                    int possible = inliers + count - index - 1;
                    if (possible < best_count || (possible == best_count && error >= best_error)) {
                        inliers = -1;
                        break;
                    }
                }
            }
            if (inliers > best_count || (inliers == best_count && error < best_error)) {
                best_count = inliers;
                best_error = error;
                *intercept = trial_intercept;
                *slope = trial_slope;
            }
        }
    }
    if (best_count < 7 || best_count * 2 < count) return 0;
    for (int iteration = 0; iteration < 2; iteration++) {
        float sum_along = 0, sum_offset = 0, sum_squared = 0, sum_product = 0;
        float first = 1, last = -1;
        int inliers = 0;
        for (int index = 0; index < count; index++) {
            float along = samples[index].along, offset = samples[index].offset;
            if (fabsf(offset - *intercept - *slope * along) >= 1.5f) continue;
            inliers++;
            sum_along += along;
            sum_offset += offset;
            sum_squared += along * along;
            sum_product += along * offset;
            if (along < first) first = along;
            if (along > last) last = along;
        }
        float determinant = inliers * sum_squared - sum_along * sum_along;
        if (inliers < 7 || last - first < 0.55f || determinant < 0.01f) return 0;
        *slope = (inliers * sum_product - sum_along * sum_offset) / determinant;
        *intercept = (sum_offset - *slope * sum_along) / inliers;
    }
    return 1;
}

static int dm_refine_original(const Image *image, BarcodeResult *result, const XcrDeadline *deadline, float search_fraction)
{
    int *corners = result->corners;
    result->geometry_quality = XCR_GEOMETRY_COARSE;
    result->geometry_edge_mask = 0;
    result->geometry_support = 0;
    result->geometry_residual = 0;
    result->subpixel_valid = 0;
    float center_x = 0, center_y = 0;
    float lines[4][3];
    for (int point = 0; point < 4; point++) {
        center_x += corners[point * 2] * 0.25f;
        center_y += corners[point * 2 + 1] * 0.25f;
    }
    for (int side = 0; side < 4; side++) {
        int next = (side + 1) % 4;
        float delta_x = corners[next * 2] - corners[side * 2];
        float delta_y = corners[next * 2 + 1] - corners[side * 2 + 1];
        float length = sqrtf(delta_x * delta_x + delta_y * delta_y);
        if (length < 32) return 0;
        float normal_x = -delta_y / length, normal_y = delta_x / length;
        float tangent_x = delta_x / length, tangent_y = delta_y / length;
        float midpoint_x = (corners[next * 2] + corners[side * 2]) * 0.5f;
        float midpoint_y = (corners[next * 2 + 1] + corners[side * 2 + 1]) * 0.5f;
        if ((center_x - midpoint_x) * normal_x + (center_y - midpoint_y) * normal_y > 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }
        int radius = (int)fminf(24, fmaxf(4, length * search_fraction));
        DmEdgeSample samples[32];
        int count = 0;
        int modules = side == 0 ? result->grid_cols : result->grid_rows;
        if (side < 2 && (modules < 10 || modules > 144 || modules % 2)) return 0;
        int groups = modules / 2 < 6 ? modules / 2 : 6;
        int sample_count = side < 2 ? groups * 5 : 32;
        for (int sample = 0; sample < sample_count; sample++) {
            if ((sample & 7) == 0 && xcr_deadline_expired(deadline)) return 0;
            float along = (sample + 0.5f) / sample_count - 0.5f;
            if (side < 2) {
                int module = 2 * ((sample / 5) * (modules / 2 - 1) / (groups - 1)) + side;
                along = (module + 0.2f + (sample % 5) * 0.15f) / modules - 0.5f;
            }
            float base_x = midpoint_x + along * delta_x, base_y = midpoint_y + along * delta_y;
            float extent_x = (radius + 2.5f) * fabsf(normal_x) + fabsf(tangent_x);
            float extent_y = (radius + 2.5f) * fabsf(normal_y) + fabsf(tangent_y);
            int interior = base_x - extent_x >= 1 && base_y - extent_y >= 1 &&
                           base_x + extent_x < image->width - 2 && base_y + extent_y < image->height - 2;
            float profile[54];
            for (int index = 0; index < radius * 2 + 6; index++) {
                float offset = index - radius - 2.5f;
                float horizontal = base_x + offset * normal_x;
                float vertical = base_y + offset * normal_y;
                if (interior) {
                    float before = dm_gray_unchecked(image, horizontal - tangent_x, vertical - tangent_y);
                    float after = dm_gray_unchecked(image, horizontal + tangent_x, vertical + tangent_y);
                    profile[index] = (before + after) * 0.5f;
                } else {
                    float before = dm_gray_at(image, horizontal - tangent_x, vertical - tangent_y);
                    float after = dm_gray_at(image, horizontal + tangent_x, vertical + tangent_y);
                    profile[index] = before < 0 || after < 0 ? -1 : (before + after) * 0.5f;
                }
            }
            float scores[49];
            float best_score = 0;
            int best = -1;
            for (int offset = -radius; offset <= radius; offset++) {
                float outside = (profile[offset + radius + 3] + 2 * profile[offset + radius + 4] + profile[offset + radius + 5]) * 0.25f;
                float inside = (profile[offset + radius] + 2 * profile[offset + radius + 1] + profile[offset + radius + 2]) * 0.25f;
                float score = outside < 0 || inside < 0 || profile[offset + radius] < 0 || profile[offset + radius + 5] < 0 ? -1000 : outside - inside;
                scores[offset + radius] = score;
                float ranked = score * (1 - 0.008f * abs(offset));
                if (score >= fmaxf(3, outside * 0.08f) && ranked > best_score) { best_score = ranked; best = offset + radius; }
            }
            if (best <= 0 || best >= radius * 2) continue;
            float weighted_offset = 0, total_weight = 0;
            for (int neighbor = -2; neighbor <= 2; neighbor++) {
                int index = best + neighbor;
                if (index < 0 || index > radius * 2) continue;
                float weight = fmaxf(0, scores[index] - scores[best] * 0.25f);
                weighted_offset += neighbor * weight;
                total_weight += weight;
            }
            float correction = total_weight > 0 ? weighted_offset / total_weight : 0;
            samples[count++] = (DmEdgeSample){along, best - radius + correction};
        }
        float intercept = 0, slope = 0;
        if (!dm_fit_edge(samples, count, &intercept, &slope)) return 0;
        unsigned support = 0;
        float residual_sum = 0;
        for (int sample = 0; sample < count; sample++) {
            float residual = fabsf(samples[sample].offset - intercept - slope * samples[sample].along);
            if (residual < 1.5f) { support++; residual_sum += residual; }
        }
        result->geometry_edge_mask |= 1U << side;
        result->geometry_support += support;
        result->geometry_residual = fmaxf(result->geometry_residual, residual_sum / support);
        lines[side][0] = normal_x - slope * delta_x / (length * length);
        lines[side][1] = normal_y - slope * delta_y / (length * length);
        lines[side][2] = lines[side][0] * midpoint_x + lines[side][1] * midpoint_y + intercept;
    }
    int refined[8];
    for (int point = 0; point < 4; point++) {
        float *previous = lines[(point + 3) % 4], *current = lines[point];
        float determinant = previous[0] * current[1] - previous[1] * current[0];
        if (fabsf(determinant) < 0.2f) return 0;
        float horizontal = (previous[2] * current[1] - previous[1] * current[2]) / determinant;
        float vertical = (previous[0] * current[2] - previous[2] * current[0]) / determinant;
        if (!isfinite(horizontal) || !isfinite(vertical) || horizontal < 0 || vertical < 0 ||
            horizontal >= image->width || vertical >= image->height ||
            fabsf(horizontal - corners[point * 2]) > 32 || fabsf(vertical - corners[point * 2 + 1]) > 32) return 0;
        refined[point * 2] = (int)lroundf(horizontal);
        refined[point * 2 + 1] = (int)lroundf(vertical);
        result->subpixel_corners[point * 2] = horizontal;
        result->subpixel_corners[point * 2 + 1] = vertical;
    }
    for (int point = 0; point < 4; point++) {
        int next = (point + 1) % 4, after = (point + 2) % 4;
        float old_cross = (corners[next * 2] - corners[point * 2]) * (corners[after * 2 + 1] - corners[next * 2 + 1]) -
                          (corners[next * 2 + 1] - corners[point * 2 + 1]) * (corners[after * 2] - corners[next * 2]);
        float new_cross = (refined[next * 2] - refined[point * 2]) * (refined[after * 2 + 1] - refined[next * 2 + 1]) -
                          (refined[next * 2 + 1] - refined[point * 2 + 1]) * (refined[after * 2] - refined[next * 2]);
        if (old_cross * new_cross <= 0) return 0;
    }
    if (xcr_deadline_expired(deadline)) return 0;
    memcpy(corners, refined, sizeof(refined));
    result->geometry_quality = XCR_GEOMETRY_REFINED;
    result->subpixel_valid = 1;
    return 1;
}

static int dm_refine_seeded(const Image *image, BarcodeResult *result, const XcrDeadline *deadline)
{
    if (result->geometry_seed_valid) {
        int coarse[8];
        memcpy(coarse, result->corners, sizeof(coarse));
        memcpy(result->corners, result->geometry_seed, sizeof(result->corners));
        if (dm_refine_original(image, result, deadline, 0.05f)) return 1;
        if (dm_refine_original(image, result, deadline, 0.08f)) return 1;
        memcpy(result->corners, coarse, sizeof(coarse));
    }
    return dm_refine_original(image, result, deadline, 0.08f);
}

int xcr_datamatrix_refine(const XcrGrayImage *image, XcrBarcodeResult *result)
{
    if (!image || !result || !image->pixels || !result->geometry_valid ||
        result->symbology != XCR_SYM_DATAMATRIX || image->width < 2 || image->height < 2 ||
        (image->stride && image->stride != image->width) ||
        image->sequence != result->sequence || image->timestamp_us != result->timestamp_us ||
        image->capture_generation != result->capture_generation) return 0;
    Image source = {.width = image->width, .height = image->height, .pixels = (uint8_t *)image->pixels};
    BarcodeResult geometry = {0};
    memcpy(geometry.corners, result->corners, sizeof(geometry.corners));
    geometry.grid_rows = result->grid_rows;
    geometry.grid_cols = result->grid_cols;
    geometry.geometry_seed_valid = result->geometry_seed_valid;
    memcpy(geometry.geometry_seed, result->geometry_seed, sizeof(geometry.geometry_seed));
    XcrDeadline deadline = {.deadline_us = image->deadline_us};
    int refined = dm_refine_seeded(&source, &geometry, &deadline);
    memcpy(result->corners, geometry.corners, sizeof(result->corners));
    result->geometry_quality = geometry.geometry_quality;
    result->geometry_edge_mask = geometry.geometry_edge_mask;
    result->geometry_support = geometry.geometry_support;
    result->geometry_residual = geometry.geometry_residual;
    result->subpixel_valid = geometry.subpixel_valid;
    memcpy(result->subpixel_corners, geometry.subpixel_corners, sizeof(result->subpixel_corners));
    return refined;
}

int xcr_datamatrix_marked_decode_ex(const Image *image, BarcodeResult *result, const XcrDeadline *deadline, int defer_geometry)
{
    uint64_t started = xcr_port_time_us();
    float axes[6] = {0};
    int roi[4];
    int orientation = 0;
    int full_retry = 0;
    memset(&last_stats, 0, sizeof(last_stats));
    datamatrix_debug_stats_reset();
search_marker:
    memset(result, 0, sizeof(*result));
    memset(axes, 0, sizeof(axes));
    uint64_t prescreen_started = xcr_port_time_us();
    int found = full_retry ? find_marker_roi(image, roi, &orientation, axes) :
                             find_marker_roi_parallel(image, roi, &orientation, axes);
    last_stats.prescreen_us += xcr_port_time_us() - prescreen_started;
    last_stats.marker_roi_valid = found;
    if (!found || xcr_deadline_expired(deadline)) {
        /* Use the native finder periodically when the external marker gate
         * misses a symbol; the fast path still owns the ordinary cadence. */
        static unsigned standard_probe;
        if (!found && !xcr_deadline_expired(deadline) &&
            (++standard_probe & 3U) == 0) {
            uint64_t probe_start = xcr_port_time_us();
            int decoded = datamatrix_decode_aligned(image, result);
            last_stats.decode_us = xcr_port_time_us() - probe_start;
            if (decoded > 0 && result->geometry_valid) {
                result->geometry_quality = XCR_GEOMETRY_COARSE;
                last_stats.total_us = xcr_port_time_us() - started;
                return decoded;
            }
        }
        last_stats.total_us = xcr_port_time_us() - started;
        return 0;
    }
    last_stats.angle_degrees = atan2f(axes[3], axes[2]) * 57.2957795f;
    memcpy(last_stats.marker_roi, roi, sizeof(roi));
    int decoded = 0;
    for (int attempt = 0; attempt < 2 && !xcr_deadline_expired(deadline); attempt++) {
    uint64_t sample_start = xcr_port_time_us();
    last_stats.normalize_attempts++;
    const int side = attempt ? 192 : XCR_DM_NORMALIZED_SIDE;
    const float margin = attempt ? side * 0.05f : 0;
    const float spacing = attempt ? side * 0.9f : side;
    const float center_offset = attempt ? 0 : -marker_step(image) * 0.5f;
    XcrMemClass workspace_class = XCR_MEM_LARGE;
#ifdef ESP_PLATFORM
    if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) > side * side + side * 8 + 32768)
        workspace_class = XCR_MEM_FAST;
#endif
    uint8_t *pixels = xcr_alloc(side * side + side * 8, workspace_class);
    if (!pixels) return 0;
    unsigned histogram[256] = {0};
    uint32_t intensity_sum = 0;
    bool transpose = fabsf(axes[2]) < fabsf(axes[4]);
    int inner_axis = transpose ? 4 : 2;
    int outer_axis = transpose ? 2 : 4;
    int32_t step_x = (int32_t)lroundf(axes[inner_axis] * (65536.0f / spacing));
    int32_t step_y = (int32_t)lroundf(axes[inner_axis + 1] * (65536.0f / spacing));
    int32_t *row_x = (int32_t *)(pixels + side * side);
    int32_t *row_y = row_x + side;
    for (int row = 0; row < side; row++) {
        float vertical = (row - margin) / spacing;
        row_x[row] = (int32_t)lroundf((axes[0] + center_offset - axes[inner_axis] * margin / spacing + vertical * axes[outer_axis]) * 65536.0f);
        row_y[row] = (int32_t)lroundf((axes[1] + center_offset - axes[inner_axis + 1] * margin / spacing + vertical * axes[outer_axis + 1]) * 65536.0f);
    }
    for (int row_block = 0; row_block < side; row_block += 16) {
      for (int column_block = 0; column_block < side; column_block += 16) {
       for (int row = row_block; row < row_block + 16; row++) {
        int32_t source_x_fixed = row_x[row];
        int32_t source_y_fixed = row_y[row];
        source_x_fixed += column_block * step_x;
        source_y_fixed += column_block * step_y;
        for (int column = column_block; column < column_block + 16; column++) {
            int source_x = (source_x_fixed + 32768) >> 16;
            int source_y = (source_y_fixed + 32768) >> 16;
            source_x_fixed += step_x;
            source_y_fixed += step_y;
            uint8_t value = source_x >= 0 && source_x < image->width && source_y >= 0 && source_y < image->height
                ? (attempt ? marker_filtered_gray(image, source_x, source_y) : image->pixels[source_y * image->width + source_x]) : 255;
            if (!attempt && value < 48) value = marker_filtered_gray(image, source_x, source_y);
            pixels[transpose ? column * side + row : row * side + column] = value;
            histogram[value]++;
            intensity_sum += value;
        }
      }
      }
    }
    trace_marker_pgm(attempt ? "05_normalized_gray_192.pgm" : "05_normalized_gray_128.pgm",
                     pixels, side, side, 0);
    unsigned dark_count = 0;
    uint32_t dark_sum = 0;
    float best_variance = -1.0f;
    int threshold = 128;
    for (int value = 0; value < 255; value++) {
        dark_count += histogram[value];
        dark_sum += (uint32_t)value * histogram[value];
        unsigned light_count = side * side - dark_count;
        if (!dark_count || !light_count) continue;
        float difference = (float)dark_sum / dark_count - (float)(intensity_sum - dark_sum) / light_count;
        float variance = (float)dark_count * light_count * difference * difference;
        if (variance > best_variance) { best_variance = variance; threshold = value; }
    }
    for (int pixel = 0; pixel < side * side; pixel++) pixels[pixel] = pixels[pixel] <= threshold ? 0 : 255;
    trace_marker_pgm(attempt ? "06_normalized_binary_192.pgm" : "06_normalized_binary_128.pgm",
                     pixels, side, side, 0);
    last_stats.normalize_pass_us[attempt] = xcr_port_time_us() - sample_start;
    last_stats.setup_us += last_stats.normalize_pass_us[attempt];
    Image normalized = {.width = side, .height = side, .pixels = pixels};
    uint64_t core_start = xcr_port_time_us();
    decoded = xcr_deadline_expired(deadline) ? 0 : datamatrix_decode_aligned(&normalized, result);
    last_stats.decode_us += xcr_port_time_us() - core_start;
    if (decoded > 0 && result->geometry_valid) {
        for (int point = 0; point < 4; point++) {
            float horizontal = (result->corners[point * 2] - margin) / spacing;
            float vertical = (result->corners[point * 2 + 1] - margin) / spacing;
            result->corners[point * 2] = (int)lroundf(axes[0] + center_offset + horizontal * axes[2] + vertical * axes[4]);
            result->corners[point * 2 + 1] = (int)lroundf(axes[1] + center_offset + horizontal * axes[3] + vertical * axes[5]);
            if (result->grid_rows == 10 && result->grid_cols == 10) {
                horizontal = horizontal < 0.5f ? 2.0f / 14 : 12.0f / 14;
                vertical = vertical < 0.5f ? 2.0f / 14 : 12.0f / 14;
                float mapped_offset = -marker_step(image) * 0.5f;
                result->geometry_seed[point * 2] = (int)lroundf(axes[0] + mapped_offset + horizontal * axes[2] + vertical * axes[4]);
                result->geometry_seed[point * 2 + 1] = (int)lroundf(axes[1] + mapped_offset + horizontal * axes[3] + vertical * axes[5]);
                result->geometry_seed_valid = 1;
            }
        }
    }
    xcr_free(pixels);
    if (decoded > 0) break;
    }
    /* A partial candidate in one half can pass the marker geometry checks
     * yet fail ECC200. Retry the original full-frame search before declaring
     * this frame unreadable, preserving the serial decoder's coverage. */
    if (!decoded && !full_retry && last_stats.marker_parallel_used &&
        !last_stats.marker_full_fallback && !xcr_deadline_expired(deadline)) {
        full_retry = 1;
        last_stats.marker_full_fallback = 1;
        goto search_marker;
    }
    if (decoded > 0 && result->geometry_valid) result->geometry_quality = XCR_GEOMETRY_COARSE;
    if (decoded > 0 && result->geometry_valid && !defer_geometry) {
        uint64_t geometry_start = xcr_port_time_us();
        last_stats.geometry_refined = dm_refine_seeded(image, result, deadline);
        last_stats.geometry_us = xcr_port_time_us() - geometry_start;
    }
    last_stats.total_us = xcr_port_time_us() - started;
    return decoded;
}

int xcr_datamatrix_marked_decode(const Image *image, BarcodeResult *result, const XcrDeadline *deadline)
{
    return xcr_datamatrix_marked_decode_ex(image, result, deadline, 0);
}
