#include "scan_primitives.h"

#include <stddef.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

static const int kDirTable[24] = {
    0, 49, 97, 142, 181, 212, 236, 251,
    256, 251, 236, 212, 181, 142, 97, 49,
    0, -49, -97, -142, -181, -212, -236, -251,
};

static const int kProbeCount[8] = {21, 21, 20, 19, 15, 19, 20, 21};

static int iabs_i32(int v) {
    return (v < 0) ? -v : v;
}

static int larger_i32(int a, int b) {
    return (a > b) ? a : b;
}

static int div_round_i64(int64_t num, int den) {
    if (den == 0) {
        return 0;
    }
    int sign = 1;
    int64_t anum = num;
    int aden = den;
    if (anum < 0) {
        anum = -anum;
        sign = -sign;
    }
    if (aden < 0) {
        aden = -aden;
        sign = -sign;
    }
    int64_t q = (anum + aden / 2) / aden;
    return (sign < 0) ? -(int)q : (int)q;
}

int sub_13AA6C(ScanContext *ctx, int center_x, int row_y, int direction_index)
{
    if (!ctx || !ctx->runtime || !ctx->raster || !ctx->raster->pixels) {
        return 0;
    }

    int idx = direction_index;
    while (idx < 0) {
        idx += 16;
    }
    idx %= 16;

    ScanLineIter line;
    int dx = kDirTable[idx];
    int dy = -kDirTable[idx + 8];
    if (!fmdclsymbol0049(&line, 0, 0, dx, dy, ctx->raster->row_stride)) {
        return 0;
    }

    const uint8_t *row = ctx->raster->pixels + (ptrdiff_t)row_y * ctx->raster->row_stride;
    const uint8_t *left = row + center_x;
    const uint8_t *right = left;

    unsigned int left_prev = *left;
    unsigned int right_prev = *right;
    unsigned int sum = *left;
    int score = 0;

    int probes = kProbeCount[idx & 7];
    for (int i = 0; i < probes; i++) {
        int step = AHEAD(&line);
        left -= step;
        right += step;

        unsigned int left_now = *left;
        unsigned int right_now = *right;

        score += iabs_i32((int)left_now - (int)left_prev);
        score += iabs_i32((int)right_now - (int)right_prev);
        sum += left_now + right_now;

        left_prev = left_now;
        right_prev = right_now;
    }

    ctx->runtime->score_accumulator += (int)sum;
    return score * 8;
}

int Accell(ScanContext *ctx, int edge_index, int stride, int run_count)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions) {
        return 0;
    }
    if (run_count < 2 || stride <= 0) {
        return 1;
    }

    int *edges = ctx->edges->positions;
    int offset = stride + (stride & 1);
    int base = edges[edge_index + stride] - edges[edge_index];
    int min_ratio = 256;
    int max_ratio = 256;

    int left_idx = edge_index + stride + (stride & 1);
    int right_idx = edge_index + 2 * stride + (stride & 1);

    for (int i = 1; i < run_count; i++) {
        int current = edges[right_idx] - edges[left_idx];
        if (current > 2 * base || current < (base >> 1)) {
            return 0;
        }

        int low = div_round_i64((int64_t)5 * base, 6);
        int high = div_round_i64((int64_t)6 * base, 5);
        if (current < low) {
            ctx->accel_penalty += div_round_i64((int64_t)(base - current) * 6, base);
        } else if (current > high) {
            ctx->accel_penalty += div_round_i64((int64_t)(current - base) * 5, base);
        }

        int ratio = (current << 8) / base;
        if (ratio < min_ratio) {
            min_ratio = ratio;
        }
        if (ratio > max_ratio) {
            max_ratio = ratio;
        }

        base = current;
        left_idx += offset;
        right_idx += offset;
    }

    return (max_ratio + 1) - min_ratio;
}

int Centered(ScanContext *ctx, int left_index, int right_index)
{
    if (!ctx || !ctx->runtime || !ctx->edges || !ctx->edges->positions) {
        return 1;
    }
    if (ctx->runtime->mode_count > 1 || ctx->edges->count == 0) {
        return 1;
    }

    int *edges = ctx->edges->positions;
    int total = edges[ctx->edges->count - 1];

    if (!ctx->params || !ctx->params->centered_enabled) {
        ctx->center_error = 0;
        int half = total >> 1;
        int right = edges[right_index];
        if (right < half) {
            ctx->center_error = half - right;
        } else {
            int left = edges[left_index];
            if (left > half) {
                ctx->center_error = left - half;
            }
        }
        return 1;
    }

    int min_pos = div_round_i64((int64_t)ctx->params->centered_min_pct * total, 100);
    int max_pos = div_round_i64((int64_t)ctx->params->centered_max_pct * total, 100);
    return (edges[right_index] >= min_pos && edges[left_index] <= max_pos) ? 1 : 0;
}

int Bars(ScanContext *ctx, int edge_index, int pair_count)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions || pair_count < 1) {
        return 0;
    }

    int sum = 0;
    int *e = ctx->edges->positions + edge_index + 1;
    for (int i = 0; i < pair_count; i++) {
        sum += e[0] - e[-1];
        e += 2;
    }
    return sum;
}

int LsBars(ScanContext *ctx, int edge_index, int span_count, int symbol_id)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions) {
        return 0;
    }

    int *e = ctx->edges->positions;
    int minw = e[edge_index + 1] - e[edge_index];
    int maxw = minw;

    for (int i = 1; i < span_count; i++) {
        int w = e[edge_index + i + 1] - e[edge_index + i];
        if (w < minw) {
            minw = w;
        }
        if (w > maxw) {
            maxw = w;
        }
    }

    int mean = (minw + maxw) >> 1;
    if (symbol_id == 98) {
        ctx->lsbars_value_98 = mean;
    } else {
        ctx->lsbars_value_other = mean;
    }
    return maxw - minw;
}

unsigned int TMod(ScanContext *ctx, int edge_index, unsigned int total_width, int modules)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions || total_width == 0) {
        return 0;
    }

    int *e = ctx->edges->positions + edge_index;
    int span = e[2] - e[0];
    return (unsigned int)((span * modules + (int)total_width / 2) / (int)total_width);
}

int WnBit(ScanContext *ctx, int edge_index, int threshold)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions) {
        return 0;
    }

    int *e = ctx->edges->positions + edge_index;
    return (e[1] - e[0] > threshold) ? 1 : 0;
}

int WnPat(ScanContext *ctx, int edge_index, int count, int even_threshold, int odd_threshold)
{
    if (!ctx || !ctx->edges || !ctx->edges->positions || count < 1) {
        return 0;
    }

    int pattern = 0;
    int *e = ctx->edges->positions + edge_index;
    for (int i = 0; i < count; i++) {
        int threshold = (i & 1) ? odd_threshold : even_threshold;
        int width = e[i + 1] - e[i];
        pattern <<= 1;
        if (width > threshold) {
            pattern |= 1;
        }
    }
    return pattern;
}

int Reverse(ScanContext *ctx)
{
    if (!ctx || !ctx->pattern || ctx->pattern_len < 2) {
        return 0;
    }

    for (int i = 0, j = ctx->pattern_len - 1; i < j; i++, j--) {
        char tmp = ctx->pattern[i];
        ctx->pattern[i] = ctx->pattern[j];
        ctx->pattern[j] = tmp;
    }
    return 1;
}

int iDiv(unsigned int num, int den)
{
    return div_round_i64((int32_t)num, den);
}

ScanPoint pSum(ScanPoint a, ScanPoint b)
{
    ScanPoint out = {a.x + b.x, a.y + b.y};
    return out;
}

ScanPoint pDif(ScanPoint a, ScanPoint b)
{
    ScanPoint out = {a.x - b.x, a.y - b.y};
    return out;
}

ScanPoint pTimes(ScanPoint p, int scale)
{
    ScanPoint out = {p.x * scale, p.y * scale};
    return out;
}

ScanPoint pDiv(ScanPoint p, int den)
{
    ScanPoint out = {iDiv((unsigned int)p.x, den), iDiv((unsigned int)p.y, den)};
    return out;
}

ScanPoint pRotate(ScanPoint p, int scale)
{
    ScanPoint out = {-p.y * scale, p.x * scale};
    return out;
}

ScanPoint pTrim(ScanPoint p, ScanPoint trim)
{
    int den = iabs_i32(p.x) + iabs_i32(p.y);
    ScanPoint out = {0, 0};
    if (den == 0) {
        return out;
    }
    out.x = div_round_i64((int64_t)p.x * iabs_i32(trim.x), den);
    out.y = div_round_i64((int64_t)p.y * iabs_i32(trim.y), den);
    return out;
}

void pExch(ScanPoint *a, ScanPoint *b)
{
    if (!a || !b) {
        return;
    }
    ScanPoint tmp = *a;
    *a = *b;
    *b = tmp;
}

int fmdclsymbol0049(ScanLineIter *it, int x0, int y0, int x1, int y1, int row_stride)
{
    if (!it) {
        return 0;
    }

    int sx = (x1 >= x0) ? 1 : -1;
    int sy = (y1 >= y0) ? 1 : -1;

    int dx = iabs_i32(x1 - x0);
    int dy = iabs_i32(y1 - y0);
    int major_x = (dx >= dy) ? 1 : 0;

    it->x = x0;
    it->y = y0;
    it->error = (major_x ? dx : dy) >> 1;
    it->dx = dx;
    it->dy = dy;
    it->step_x = sx;
    it->step_y = sy;
    it->major_x = major_x;
    it->pixel_step = sy * row_stride;

    if (dx == 0 && dy == 0) {
        it->dx = 1;
        it->dy = 1;
        dx = 1;
        dy = 1;
    }
    return larger_i32(dx, dy);
}

int AHEAD(ScanLineIter *it)
{
    if (!it) {
        return 0;
    }

    int delta;
    if (it->major_x) {
        delta = it->step_x;
        it->x += it->step_x;
        it->error += it->dy;
        if (it->error >= it->dx) {
            it->y += it->step_y;
            it->error -= it->dx;
            delta += it->pixel_step;
        }
    } else {
        delta = it->pixel_step;
        it->y += it->step_y;
        it->error += it->dx;
        if (it->error >= it->dy) {
            it->x += it->step_x;
            it->error -= it->dy;
            delta += it->step_x;
        }
    }
    return delta;
}

int BACK(ScanLineIter *it)
{
    if (!it) {
        return 0;
    }

    int delta;
    if (it->major_x) {
        delta = -it->step_x;
        it->x -= it->step_x;
        it->error -= it->dy;
        if (it->error < 0) {
            it->y -= it->step_y;
            it->error += it->dx;
            delta -= it->pixel_step;
        }
    } else {
        delta = -it->pixel_step;
        it->y -= it->step_y;
        it->error -= it->dx;
        if (it->error <= 0) {
            it->x -= it->step_x;
            it->error += it->dy;
            delta -= it->step_x;
        }
    }
    return delta;
}

int Fwd(ScanSubpixelIter *it, int scale)
{
    if (!it || scale == 0) {
        return 0;
    }

    int half = (scale + 1) >> 1;
    if (it->phase != scale) {
        it->phase = scale;
        it->frac_x = 0;
        it->frac_y = 0;
    }

    it->frac_x += it->inc_x;
    while (it->frac_x >= half) {
        it->frac_x -= scale;
        it->x += it->dir_x;
    }

    it->frac_y += it->inc_y;
    while (it->frac_y >= half) {
        it->frac_y -= scale;
        it->y += it->dir_y;
    }

    return 1;
}

int Bak(ScanSubpixelIter *it, int scale)
{
    if (!it || scale == 0) {
        return 0;
    }

    int half_neg = -(scale >> 1);
    if (it->phase != scale) {
        it->phase = scale;
        it->frac_x = 0;
        it->frac_y = 0;
    }

    it->frac_x -= it->inc_x;
    while (it->frac_x < half_neg) {
        it->frac_x += scale;
        it->x -= it->dir_x;
    }

    it->frac_y -= it->inc_y;
    while (it->frac_y < half_neg) {
        it->frac_y += scale;
        it->y -= it->dir_y;
    }

    return 1;
}

int Angle(int x0, int y0, int x1, int y1, int units)
{
    int dx = x1 - x0;
    int dy = y1 - y0;
    if (dx == 0 && dy == 0) {
        return -1;
    }

    int adx = iabs_i32(dx);
    int ady = iabs_i32(dy);

    int ang;
    if (ady > adx) {
        int b = dy;
        int adj = dy * units + ((dx * units) >> 1);
        ang = (adj + b / 2) / b;
    } else {
        int t = (dx >= 0) ? (2 * dx * units) : (4 * dx * units);
        int b = dx;
        int adj = t - ((dy * units) >> 1);
        ang = (adj + b / 2) / b;
    }

    if (ang >= 4 * units) {
        ang -= 4 * units;
    }
    return ang;
}

ScanPoint Frame(int x, int y, int x2, int y2,
                int left, int right, int top, int bottom)
{
    int out_x = x;
    int out_y = y;

    if (x < left) {
        out_x = left;
        if (x != x2) {
            out_y += (left - x) * (y2 - y) / (x2 - x);
        }
    } else if (x > right) {
        out_x = right;
        if (x != x2) {
            out_y += (right - x) * (y2 - y) / (x2 - x);
        }
    }

    if (out_y < top) {
        if (out_y != y2) {
            out_x += (top - out_y) * (x2 - out_x) / (y2 - out_y);
        }
        out_y = top;
    } else if (out_y > bottom) {
        if (out_y != y2) {
            out_x += (bottom - out_y) * (x2 - out_x) / (y2 - out_y);
        }
        out_y = bottom;
    }

    ScanPoint p = {out_x, out_y};
    return p;
}
