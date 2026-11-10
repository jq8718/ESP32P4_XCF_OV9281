#ifndef MID1D_RESTORE_H
#define MID1D_RESTORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int x;
    int y;
} Mid1DPoint;

typedef struct {
    int x0;
    int y0;
    int x1;
    int y1;
    int span;
} Mid1DProjectLine;

typedef struct {
    const uint8_t *pixels;
    int width;
    int height;
    int stride;
    int frac_bits;
} Mid1DSampleU8;

int mid1d_fmwlsymbol000c_project(
    const Mid1DProjectLine *line,
    int offset,
    Mid1DPoint *out);

int mid1d_fmwlsymbol0022_sample_u8(
    const Mid1DSampleU8 *src,
    int x_fp,
    int y_fp,
    int *out_value);

#ifdef __cplusplus
}
#endif

#endif
