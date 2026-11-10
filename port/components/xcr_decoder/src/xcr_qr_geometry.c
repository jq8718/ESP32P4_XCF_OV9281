#include "xcr_qr_geometry.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define EDGE_SAMPLES 48
#define SEARCH_RADIUS 16
#define EDGE_INLIER_DISTANCE 2.4f

typedef struct { float along, offset; } edge_point_t;
typedef struct { float normal_x, normal_y, constant; } edge_line_t;

static float pixel_at(const uint8_t *pixels, int width, int height, int stride, float horizontal, float vertical)
{
    if (horizontal < 0 || vertical < 0 || horizontal >= width - 1 || vertical >= height - 1) return -1;
    int column = (int)horizontal, row = (int)vertical;
    float fraction_x = horizontal - column, fraction_y = vertical - row;
    const uint8_t *source = pixels + (size_t)row * stride + column;
    float upper = source[0] + fraction_x * (source[1] - source[0]);
    float lower = source[stride] + fraction_x * (source[stride + 1] - source[stride]);
    return upper + fraction_y * (lower - upper);
}

static int fit_offsets(const edge_point_t *points, int count, float *intercept, float *slope)
{
    int best_count = 0;
    float best_error = INFINITY;
    for (int first = 0; first < count; first += 2) {
        for (int second = first + 1; second < count; second += 2) {
            float separation = points[second].along - points[first].along;
            if (separation < 0.4f) continue;
            float trial_slope = (points[second].offset - points[first].offset) / separation;
            float trial_intercept = points[first].offset - trial_slope * points[first].along;
            int inliers = 0;
            float error = 0;
            for (int index = 0; index < count; index++) {
                float residual = fabsf(points[index].offset - trial_intercept - trial_slope * points[index].along);
                if (residual < EDGE_INLIER_DISTANCE) { inliers++; error += residual; }
            }
            if (inliers > best_count || (inliers == best_count && error < best_error)) {
                best_count = inliers;
                best_error = error;
                *intercept = trial_intercept;
                *slope = trial_slope;
            }
        }
    }
    if (best_count < 12 || best_count * 2 < count) return 0;
    for (int iteration = 0; iteration < 2; iteration++) {
        float sum_along = 0, sum_offset = 0, sum_squared = 0, sum_product = 0;
        float first = 1, last = -1;
        int inliers = 0;
        for (int index = 0; index < count; index++) {
            float along = points[index].along, offset = points[index].offset;
            if (fabsf(offset - *intercept - *slope * along) >= EDGE_INLIER_DISTANCE) continue;
            inliers++;
            sum_along += along;
            sum_offset += offset;
            sum_squared += along * along;
            sum_product += along * offset;
            if (along < first) first = along;
            if (along > last) last = along;
        }
        float determinant = inliers * sum_squared - sum_along * sum_along;
        if (inliers < 12 || last - first < 0.6f || determinant < 0.01f) return 0;
        *slope = (inliers * sum_product - sum_along * sum_offset) / determinant;
        *intercept = (sum_offset - *slope * sum_along) / inliers;
    }
    return 1;
}

int xcr_qr_refine_corners(const uint8_t *pixels, int width, int height, int stride,
                          int corners[8], const XcrDeadline *deadline)
{
    if (!pixels || !corners || width < 32 || height < 32 || stride < width) return 0;
    float center_x = 0, center_y = 0;
    for (int index = 0; index < 4; index++) {
        if (corners[index * 2] < 0 || corners[index * 2] >= width ||
            corners[index * 2 + 1] < 0 || corners[index * 2 + 1] >= height) return 0;
        center_x += corners[index * 2] * 0.25f;
        center_y += corners[index * 2 + 1] * 0.25f;
    }
    edge_line_t lines[4];
    for (int side = 0; side < 4; side++) {
        int next = (side + 1) % 4;
        float delta_x = corners[next * 2] - corners[side * 2];
        float delta_y = corners[next * 2 + 1] - corners[side * 2 + 1];
        float length = sqrtf(delta_x * delta_x + delta_y * delta_y);
        if (length < 40) return 0;
        float normal_x = -delta_y / length, normal_y = delta_x / length;
        float midpoint_x = (corners[side * 2] + corners[next * 2]) * 0.5f;
        float midpoint_y = (corners[side * 2 + 1] + corners[next * 2 + 1]) * 0.5f;
        if ((center_x - midpoint_x) * normal_x + (center_y - midpoint_y) * normal_y > 0) {
            normal_x = -normal_x;
            normal_y = -normal_y;
        }
        edge_point_t points[EDGE_SAMPLES];
        int count = 0;
        int radius = length < 160 ? (int)(length / 10) : SEARCH_RADIUS;
        for (int sample = 0; sample < EDGE_SAMPLES; sample++) {
            if ((sample & 7) == 0 && xcr_deadline_expired(deadline)) return 0;
            float along = (sample + 0.5f) / EDGE_SAMPLES - 0.5f;
            float base_x = midpoint_x + along * delta_x, base_y = midpoint_y + along * delta_y;
            float scores[SEARCH_RADIUS * 2 + 1];
            float best_score = 12;
            int best = -1;
            for (int offset = -radius; offset <= radius; offset++) {
                float outside = pixel_at(pixels, width, height, stride,
                                         base_x + (offset + 2) * normal_x, base_y + (offset + 2) * normal_y);
                float inside = pixel_at(pixels, width, height, stride,
                                        base_x + (offset - 2) * normal_x, base_y + (offset - 2) * normal_y);
                float score = inside < 0 || outside < 0 ? -1000 : outside - inside;
                scores[offset + radius] = score;
                float ranked = score - 0.6f * abs(offset);
                if (ranked > best_score) { best = offset + radius; best_score = ranked; }
            }
            if (best <= 0 || best >= radius * 2) continue;
            float curvature = scores[best - 1] - 2 * scores[best] + scores[best + 1];
            float correction = curvature < -0.1f ? 0.5f * (scores[best - 1] - scores[best + 1]) / curvature : 0;
            if (correction < -0.5f) correction = -0.5f;
            if (correction > 0.5f) correction = 0.5f;
            points[count++] = (edge_point_t){along, best - radius + correction};
        }
        float intercept = 0, slope = 0;
        if (!fit_offsets(points, count, &intercept, &slope)) return 0;
        lines[side].normal_x = normal_x - slope * delta_x / (length * length);
        lines[side].normal_y = normal_y - slope * delta_y / (length * length);
        lines[side].constant = lines[side].normal_x * midpoint_x + lines[side].normal_y * midpoint_y + intercept;
    }
    int refined[8];
    for (int corner = 0; corner < 4; corner++) {
        edge_line_t previous = lines[(corner + 3) % 4], current = lines[corner];
        float determinant = previous.normal_x * current.normal_y - previous.normal_y * current.normal_x;
        if (fabsf(determinant) < 0.2f) return 0;
        float horizontal = (previous.constant * current.normal_y - previous.normal_y * current.constant) / determinant;
        float vertical = (previous.normal_x * current.constant - previous.constant * current.normal_x) / determinant;
        if (!isfinite(horizontal) || !isfinite(vertical) || horizontal < 0 || vertical < 0 ||
            horizontal >= width || vertical >= height ||
            fabsf(horizontal - corners[corner * 2]) > 24 || fabsf(vertical - corners[corner * 2 + 1]) > 24) return 0;
        refined[corner * 2] = (int)lrintf(horizontal);
        refined[corner * 2 + 1] = (int)lrintf(vertical);
    }
    if (xcr_deadline_expired(deadline)) return 0;
    for (int corner = 0; corner < 4; corner++) {
        int next = (corner + 1) % 4, after = (corner + 2) % 4;
        float old_dx = corners[next * 2] - corners[corner * 2];
        float old_dy = corners[next * 2 + 1] - corners[corner * 2 + 1];
        float old_cross = old_dx * (corners[after * 2 + 1] - corners[next * 2 + 1]) -
                          old_dy * (corners[after * 2] - corners[next * 2]);
        float new_dx = refined[next * 2] - refined[corner * 2];
        float new_dy = refined[next * 2 + 1] - refined[corner * 2 + 1];
        float new_cross = new_dx * (refined[after * 2 + 1] - refined[next * 2 + 1]) -
                          new_dy * (refined[after * 2] - refined[next * 2]);
        float old_length_squared = old_dx * old_dx + old_dy * old_dy;
        float new_length_squared = new_dx * new_dx + new_dy * new_dy;
        if (old_cross * new_cross <= 0 || new_length_squared < old_length_squared * 0.49f ||
            new_length_squared > old_length_squared * 1.69f) return 0;
    }
    memcpy(corners, refined, sizeof(refined));
    return 1;
}
