#ifndef SCAN_PRIMITIVES_H
#define SCAN_PRIMITIVES_H

#include <stdint.h>

typedef struct {
    int x;
    int y;
} ScanPoint;

typedef struct {
    int x;
    int y;
    int error;
    int dx;
    int dy;
    int step_x;
    int step_y;
    int major_x;
    int pixel_step;
} ScanLineIter;

typedef struct {
    int x;
    int y;
    int frac_x;
    int frac_y;
    int inc_x;
    int inc_y;
    int dir_x;
    int dir_y;
    int phase;
} ScanSubpixelIter;

typedef struct {
    const uint8_t *pixels;
    int row_stride;
} ScanRaster;

typedef struct {
    int mode_count;
    int score_accumulator;
} ScanRuntime;

typedef struct {
    int centered_enabled;
    int centered_min_pct;
    int centered_max_pct;
} ScanParams;

typedef struct {
    int *positions;
    int count;
} ScanEdges;

typedef struct {
    ScanRuntime *runtime;
    ScanRaster *raster;
    ScanParams *params;
    ScanEdges *edges;

    int center_error;
    char *pattern;
    int pattern_len;

    int lsbars_value_98;
    int lsbars_value_other;
    int accel_penalty;
} ScanContext;

int sub_13AA6C(ScanContext *ctx, int center_x, int row_y, int direction_index);
int Accell(ScanContext *ctx, int edge_index, int stride, int run_count);
int Centered(ScanContext *ctx, int left_index, int right_index);
int Bars(ScanContext *ctx, int edge_index, int pair_count);
int LsBars(ScanContext *ctx, int edge_index, int span_count, int symbol_id);
unsigned int TMod(ScanContext *ctx, int edge_index, unsigned int total_width, int modules);
int WnBit(ScanContext *ctx, int edge_index, int threshold);
int WnPat(ScanContext *ctx, int edge_index, int count, int even_threshold, int odd_threshold);
int Reverse(ScanContext *ctx);

int iDiv(unsigned int num, int den);
ScanPoint pSum(ScanPoint a, ScanPoint b);
ScanPoint pDif(ScanPoint a, ScanPoint b);
ScanPoint pTimes(ScanPoint p, int scale);
ScanPoint pDiv(ScanPoint p, int den);
ScanPoint pRotate(ScanPoint p, int scale);
ScanPoint pTrim(ScanPoint p, ScanPoint trim);
void pExch(ScanPoint *a, ScanPoint *b);

int fmdclsymbol0049(ScanLineIter *it, int x0, int y0, int x1, int y1, int row_stride);
int AHEAD(ScanLineIter *it);
int BACK(ScanLineIter *it);
int Fwd(ScanSubpixelIter *it, int scale);
int Bak(ScanSubpixelIter *it, int scale);

int Angle(int x0, int y0, int x1, int y1, int units);
ScanPoint Frame(int x, int y, int x2, int y2,
                int left, int right, int top, int bottom);

#endif
