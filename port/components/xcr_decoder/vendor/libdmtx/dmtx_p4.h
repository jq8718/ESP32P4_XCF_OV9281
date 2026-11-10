#ifndef DMTX_P4_FLOAT_API_H
#define DMTX_P4_FLOAT_API_H
#include <time.h>
#include <sys/time.h>
#define double float
#define cos cosf
#define sin sinf
#define sqrt sqrtf
#define atan2 atan2f
#define acos acosf
#define fabs fabsf
#include "dmtx.h"
#undef fabs
#undef acos
#undef atan2
#undef sqrt
#undef sin
#undef cos
#undef double
#endif
