#ifdef _MSC_VER
#include <intrin.h>
#define __builtin_popcount __popcnt
#endif

#include "signal.h"
#include "scan_primitives.h"

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

typedef int _DWORD;

static uintptr_t dword_2081AC;
static int dword_2081B4;
static int dword_2081C8;
static int dword_2081CC;
static int dword_2081D0;
static int dword_2081D4;
static int dword_2081D8;
static int dword_2081E0;

static int SmartSymbolRedef0005ef33[128];
static int g_sinc_table_init;

static inline int smmul(int a, int b)
{
    return (int)(((int64_t)a * (int64_t)b) >> 32);
}

static void init_sinc_table(void)
{
    if (g_sinc_table_init) {
        return;
    }
    memset(SmartSymbolRedef0005ef33, 0, sizeof(SmartSymbolRedef0005ef33));

    /* phase coefficients reconstructed from SmartSymbolRedef0005ef33 access pattern */
    static const int coeff[16][4] = {
        {0, 256, 0, 0},
        {-8, 255, 10, -1},
        {-14, 249, 23, -2},
        {-17, 239, 39, -4},
        {-19, 225, 56, -6},
        {-20, 208, 76, -8},
        {-19, 188, 98, -11},
        {-17, 166, 120, -13},
        {-15, 143, 143, -15},
        {-13, 120, 166, -17},
        {-11, 98, 188, -19},
        {-8, 76, 208, -20},
        {-6, 56, 225, -19},
        {-4, 39, 239, -17},
        {-2, 23, 249, -14},
        {-1, 10, 255, -8},
    };

    for (int p = 0; p < 16; p++) {
        int base = 1 + 8 * p;
        SmartSymbolRedef0005ef33[base + 0] = coeff[p][0];
        SmartSymbolRedef0005ef33[base + 1] = coeff[p][1];
        SmartSymbolRedef0005ef33[base + 2] = coeff[p][2];
        SmartSymbolRedef0005ef33[base + 3] = coeff[p][3];
    }

    g_sinc_table_init = 1;
}

/* source: libFsaDecoderCore1.c:64205-64594 (sub_5F93C) */
static int sub_5F93C(unsigned int a1, unsigned int a2, signed int a3, uintptr_t a4, signed int a5, int a6)
{
    int v6;
    unsigned int v7;
    int v9;
    int v10;
    int v11;
    int v12;
    int v13;
    int v14;
    int v15;
    intptr_t v16;
    int v17;
    int v18;
    int v19;
    unsigned int v20;
    unsigned int v22;
    int v24;
    signed int v25;
    signed int v26;
    int v27;
    int v28;
    int v29;
    unsigned int *v30;
    int v31;
    int v32;
    int v33;
    int v34;
    int v35;
    int v36;
    int v37;
    int v38;
    intptr_t v39;
    int v40;
    int v41;
    intptr_t v42;
    int v43;
    int v44;
    int v45;
    int v46;

    if (a3 >= 1) {
        v6 = a3 + 1;
        v24 = dword_2081B4 - 4;
        v26 = -12345678;
        v25 = -12345678;
        do {
            v7 = a1 >> 10;
            v30 = (unsigned int *)a4;
            v29 = v6;
            if (v26 == (signed int)(a2 >> 10) && v25 == (signed int)v7) {
                v28 = v46;
            } else {
                const uint8_t *vbase;
                v26 = (signed int)(a2 >> 10);
                v9 = (int)(v7 + ((dword_2081C8 & (int)(a2 >> 10)) - 1) * dword_2081B4);
                vbase = (const uint8_t *)dword_2081AC + v9;
                v31 = vbase[-1];
                v32 = vbase[0];
                v33 = vbase[1];
                v34 = vbase[2];
                v35 = vbase[v24 + 3];
                v36 = vbase[v24 + 4];
                v37 = vbase[v24 + 5];
                v10 = v9 - 1 + v24 + 4 + v24;
                v38 = ((const uint8_t *)dword_2081AC)[v9 - 1 + v24 + 7];
                v11 = ((const uint8_t *)dword_2081AC)[v10 + 4];
                v10 += 4;
                v39 = v11;
                v40 = ((const uint8_t *)dword_2081AC)[v10 + 1];
                v41 = ((const uint8_t *)dword_2081AC)[v10 + 2];
                v12 = ((const uint8_t *)dword_2081AC)[v10 + 3];
                v13 = v10 + v24;
                v42 = v12;
                v43 = ((const uint8_t *)dword_2081AC)[v13 + 4];
                v44 = ((const uint8_t *)dword_2081AC)[v13 + 5];
                v45 = ((const uint8_t *)dword_2081AC)[v13 + 6];
                v28 = ((const uint8_t *)dword_2081AC)[v13 + 7];
                v46 = ((const uint8_t *)dword_2081AC)[v13 + 7];
                v25 = (int)(a1 >> 10);
            }
            v14 = (a1 >> 1) & 0x1E0;
            v15 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v14 + 4);
            v16 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v14 + 8);
            v17 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v14 + 12);
            v18 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v14 + 16);
            v27 = v39 * v15 + v40 * v16 + v41 * v17 + v42 * v18;
            v19 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + ((a2 >> 1) & 0x1E0) + 4)
                * (v31 * v15 + v32 * v16 + v33 * v17 + v34 * v18)
                + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + ((a2 >> 1) & 0x1E0) + 8)
                * (v35 * v15 + v36 * v16 + v37 * v17 + v38 * v18)
                + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + ((a2 >> 1) & 0x1E0) + 12) * v27
                + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + ((a2 >> 1) & 0x1E0) + 16)
                * (v43 * v15 + v44 * v16 + v45 * v17 + v28 * v18)
                + 0x8000;
            v20 = 0xFFFFFF;
            if (v19 < 0xFFFFFF) {
                v20 = (unsigned int)v19;
            }
            v22 = 0;
            if (v19 > 0) {
                v22 = v20 >> 16;
            }
            *v30 = v22;
            a4 = (uintptr_t)(v30 + 1);
            a2 += (unsigned int)a6;
            a3 = a5;
            v6 = v29 - 1;
            a1 += (unsigned int)a5;
        } while (v29 - 1 > 1);
    }
    return 0;
}

/* source: src/decompiled_signal/sample_signal_0006016140.c:1-274 */
static int SmartSymbolRedef0006016140(int *a1, float _R1f, float _R2f, float _R3f, float a5, signed int a6)
{
    uintptr_t v6;
    signed int v10;
    signed int v11;
    int v12;
    int v16;
    int v17;
    int v18;
    signed int v20;
    int v21;
    int v22;
    unsigned int v23;
    signed int v24;
    signed int v25;
    signed int *v26;
    int v27;
    int v29;
    int v30;
    int v31;
    int v32;
    int v33;
    int v34;
    signed int v35;
    int v36;
    int v37;
    const uint8_t *v38;
    intptr_t v39;
    int v40;
    int v41;
    intptr_t v42;
    signed int *v43;
    int v45;
    signed int v46;
    signed int v47;
    signed int v48;
    int v49;
    signed int v50;
    signed int v51;
    int v52;
    int v53;
    int v54;
    int v55;
    int v56;
    int v57;
    int v58;
    int v59;
    int v60;
    int v61;
    int v62;
    int v63;
    int v64;
    int v65;
    int v66;
    int v67;
    int v68;

    init_sinc_table();

    v6 = (uintptr_t)a1;

    /* original ASM:
       VMOV/VLDR/VMUL/VCVTR/VMOV block for fixed-point setup */
    int _R10 = (int)lrintf(_R3f * 1024.0f); /* step x in Q10 */
    int _R1 = (int)lrintf(_R2f * 1024.0f);  /* y in Q10 */
    int _R7 = (int)lrintf(_R1f * 1024.0f);  /* x in Q10 */
    v52 = (int)lrintf(a5 * 1024.0f);        /* step y in Q10 */

    v10 = 1;
    v11 = 1;
    v12 = (dword_2081D4 << 10) + 2048;
    if (v12 <= _R7) {
        v16 = (dword_2081D8 << 10) - 2048;
        v11 = 1;
        if (v16 >= _R7) {
            if (dword_2081E0) {
                v17 = (dword_2081CC << 10) + 2048;
                v11 = 1;
                if (v17 <= _R1) {
                    v18 = (dword_2081D0 << 10) - 2048;
                    v11 = 1;
                    if (v18 >= _R1) {
                        v11 = 0;
                    }
                }
            } else {
                v11 = 0;
            }
        }
    }

    v20 = _R7;
    v21 = _R7 + (a6 - 1) * _R10;
    if (v12 <= v21) {
        if ((dword_2081D8 << 10) - 2048 >= v21) {
            if (!dword_2081E0) {
                v23 = (unsigned int)v11;
                if (v11) {
                    goto LABEL_17;
                }
                sub_5F93C((unsigned int)_R7, (unsigned int)_R1, a6, v6, _R10, v52);
                return 0;
            }
            v22 = _R1 + (a6 - 1) * v52;
            if ((dword_2081CC << 10) + 2048 <= v22) {
                if ((dword_2081D0 << 10) - 2048 >= v22) {
                    v10 = 0;
                }
            }
        }
    }
    v23 = (unsigned int)(v10 + v11);
    if (!(v10 + v11)) {
        sub_5F93C((unsigned int)_R7, (unsigned int)_R1, a6, v6, _R10, v52);
        return 0;
    }

LABEL_17:
    if (v23 != 1) {
        if (v23 == 2 && a6 >= 1) {
            memset((void *)v6, 0, (size_t)(4 * a6));
        }
        return 0;
    }
    if (a6 < 1) {
        return 0;
    }

    v45 = dword_2081B4 - 4;
    v24 = 256;
    v25 = a6;
    v26 = (signed int *)v6;
    v47 = -12345678;
    v48 = -12345678;
    v46 = _R10;
    do {
        v27 = v20 >> 10;
        if (v48 == (_R1 >> 10) && v47 == v27) {
            v50 = v25;
            v51 = v24;
            v49 = v68;
        } else {
            if (v27 <= dword_2081D4
                || v27 + 2 >= dword_2081D8
                || ((v36 = _R1 >> 10), dword_2081E0)
                && (v36 <= dword_2081CC || v36 + 2 >= dword_2081D0)) {
                *v26 = -1;
                goto LABEL_42;
            }
            v50 = v25;
            v51 = v24;
            v37 = v27 + ((dword_2081C8 & v36) - 1) * dword_2081B4;
            v38 = (const uint8_t *)dword_2081AC + v37 - 1;
            v53 = v38[0];
            v54 = ((const uint8_t *)dword_2081AC)[v37 + 0];
            v55 = ((const uint8_t *)dword_2081AC)[v37 + 1];
            v56 = ((const uint8_t *)dword_2081AC)[v37 + 2];
            v57 = v38[v45 + 4];
            v58 = v38[v45 + 5];
            v59 = v38[v45 + 6];
            v39 = (intptr_t)(v38 + v45 + 4 + v45);
            v60 = v38[v45 + 7];
            v40 = *(const uint8_t *)(v39 + 4);
            v39 += 4;
            v61 = v40;
            v62 = *(const uint8_t *)(v39 + 1);
            v63 = *(const uint8_t *)(v39 + 2);
            v41 = *(const uint8_t *)(v39 + 3);
            v42 = v39 + v45;
            v64 = v41;
            v65 = *(const uint8_t *)(v42 + 4);
            v66 = *(const uint8_t *)(v42 + 5);
            v67 = *(const uint8_t *)(v42 + 6);
            v49 = *(const uint8_t *)(v42 + 7);
            v68 = *(const uint8_t *)(v42 + 7);
            v48 = _R1 >> 10;
            v47 = v20 >> 10;
        }
        v29 = ((unsigned int)v20 >> 1) & 0x1E0;
        v30 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v29 + 4);
        v31 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v29 + 8);
        v32 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v29 + 12);
        v33 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v29 + 16);
        v34 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + (((_R1 >> 1) & 0x1E0) + 4))
            * (v53 * v30 + v54 * v31 + v55 * v32 + v56 * v33)
            + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + (((_R1 >> 1) & 0x1E0) + 8))
            * (v57 * v30 + v58 * v31 + v59 * v32 + v60 * v33)
            + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + (((_R1 >> 1) & 0x1E0) + 12))
            * (v61 * v30 + v62 * v31 + v63 * v32 + v64 * v33)
            + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + (((_R1 >> 1) & 0x1E0) + 16))
            * (v65 * v30 + v66 * v31 + v67 * v32 + v49 * v33)
            + 0x8000;
        v23 = 0xFFFFFF;
        if (v34 < 0xFFFFFF) {
            v23 = (unsigned int)v34;
        }
        v35 = 0;
        if (v34 > 0) {
            v35 = (signed int)(v23 >> 16);
        }
        *v26 = v35;
        v24 = v51;
        _R10 = v46;
        v25 = v50;
        if (v35 < v51) {
            v24 = v35;
        }
LABEL_42:
        v20 += _R10;
        --v25;
        ++v26;
        _R1 += v52;
    } while (v25);

    v43 = (signed int *)v6;
    if (a6 >= 1) {
        int _R2 = a6;
        do {
            if (*v43 == -1) {
                *v43 = v24;
            }
            --_R2;
            ++v43;
        } while (_R2);
    }

    return 0;
}

/* source: src/decompiled_signal/envelope_threshold_000601676.c:1-105 */
static int *SmartSymbolRedef000601676(int *result, signed int a2, _DWORD *a3, _DWORD *a4)
{
    int v4;
    int v5;
    int v6;
    int v7;
    int *v8;
    int v9;
    int v10;
    int v11;
    int v12;
    int v13;
    int v14;
    int v15;
    int v16;
    int v17;
    int v18;
    int v19;
    _DWORD *v20;
    _DWORD *v21;

    v4 = *result;
    v5 = 0;
    if (!(a2 >> 2)) {
        v7 = *result;
        v6 = *result;
        v15 = a2 & 3;
        if (!v15) {
            goto LABEL_37;
        }
        goto LABEL_30;
    }
    v20 = a3;
    v21 = a4;
    v6 = *result;
    v7 = *result;
    while (1) {
        v8 = &result[4 * v5];
        v9 = v6;
        if (v4 < v6) {
            v9 = v4;
        }
        v10 = v8[1];
        v11 = v8[2];
        if (v4 > v7) {
            v9 = v6;
        }
        v12 = v9;
        v13 = v8[3];
        if (v10 < v9) {
            v12 = v8[1];
        }
        if (v4 > v7) {
            v7 = v4;
        }
        if (v10 > v7) {
            v12 = v9;
        }
        v14 = v12;
        if (v11 < v12) {
            v14 = v8[2];
        }
        if (v10 > v7) {
            v7 = v8[1];
        }
        if (v11 > v7) {
            v14 = v12;
        }
        v6 = v14;
        if (v13 < v14) {
            v6 = v8[3];
        }
        if (v11 > v7) {
            v7 = v8[2];
        }
        if (v13 > v7) {
            v6 = v14;
            v7 = v8[3];
        }
        if ((a2 >> 2) - 1 == v5) {
            break;
        }
        v4 = v8[4];
        ++v5;
    }
    result += 4 * (a2 >> 2);
    a3 = v20;
    a4 = v21;
    v15 = a2 & 3;
    if (v15) {
LABEL_30:
        v16 = -v15;
        do {
            v18 = *result;
            ++result;
            v17 = v18;
            v19 = v6;
            if (v18 < v6) {
                v19 = v17;
            }
            if (v17 > v7) {
                v7 = v17;
            } else {
                v6 = v19;
            }
            ++v16;
        } while (v16);
    }
LABEL_37:
    *a3 = v6;
    *a4 = v7;
    return result;
}

/* source: src/decompiled_signal/find_transitions_0006016143.c:1-188 */
static signed int SmartSymbolRedef0006016143(int *a1, _DWORD *a2, int a3, int a4, int a5, signed int a6)
{
    signed int v6;
    signed int v14;
    int v15;
    uintptr_t v16;
    int v17;
    int *v18;
    uintptr_t v19;
    int v20;
    int v21;
    int v22;
    int v23;
    int *v24;
    int v28;
    int v29;
    intptr_t v30;
    int v31;
    int v32;
    int v33;
    signed int v34;
    int v35;
    int *v36;
    int v40;
    int v41;
    uintptr_t v42;
    int v43;
    int v44;
    int v45;
    signed int v46;
    int v47;
    int v49;
    int v50;
    int v51;
    uintptr_t v52;
    int v53;
    int *v54;
    _DWORD *v55;
    int v56;
    int v57;
    int *v58;
    int *v59;

    v6 = 0;
    v55 = a2;
    v54 = a1;
    if (a3 > a4) {
        int _R1 = smmul(a3 - a4, 715827883);
        *v55 = 0;
        v14 = a6;
        v15 = *a1;
        v57 = _R1 + (_R1 >> 31);
        v16 = (uintptr_t)&a1[a5 - 1];
        v56 = a4 + a3;
        v17 = (a4 + a3) >> 1;
        v51 = 0;
        if (a6) {
            v14 = 1;
        }
        v6 = 1;
        v18 = v54;
        v59 = v54;
        v58 = v54;
        v19 = v16;
        v50 = 0;
        v49 = (a4 + a3) >> 1;
        v53 = a3 - a4;
        v52 = v16;
        do {
            while (1) {
                v20 = v15;
                v21 = v18[1];
                ++v18;
                v15 = v21;
                v22 = v20 - v56 + v21;
                if (v22 < 0) {
                    v22 = -v22;
                }
                v23 = v22 * v57 / (a3 - a4) + 4;
                if (v14) {
                    break;
                }
                v36 = v58;
                if (v15 < v17) {
                    v36 = v18;
                    v17 = v15;
                }
                v58 = v36;
                if ((uintptr_t)v18 == v19 || (v14 = 0, v15 > v17 + v23)) {
                    if (v6 >= 2) {
                        int _R0 = smmul(v51 + v49 + v17, 1431655766);
                        v41 = _R0 + (_R0 >> 31);
                        v40 = v41;
                        if (v41 > v51) {
                            v41 = v51;
                        }
                        if (v40 <= v17) {
                            v41 = v17 + 1;
                        }
                        v42 = (uintptr_t)(v59 - 1);
                        do {
                            v43 = *(int *)(v42 + 4);
                            v42 += 4;
                        } while (v43 >= v41);
                        v44 = (int)(4 * (v42 - (uintptr_t)v54));
                        v55[v6 - 1] = v44;
                        if (v44) {
                            v45 = *(int *)v42;
                            v46 = *(int *)(v42 - 4) - *(int *)v42;
                            if (v46) {
                                v47 = 16 * (v41 - v45) / v46;
                            } else {
                                v47 = 8;
                            }
                            v55[v6 - 1] = v44 - v47;
                        }
                    }
                    v14 = 1;
                    v55[v6++] = (int)((char *)v18 - (char *)v54) * 4;
                    v50 = v17;
                    v17 = v15;
                    v59 = v18;
                    goto LABEL_44;
                }
LABEL_26:
                if ((uintptr_t)v18 >= v19) {
                    return v6;
                }
            }
            v24 = v59;
            if (v15 > v17) {
                v24 = v18;
                v17 = v15;
            }
            v59 = v24;
            if ((uintptr_t)v18 != v19 && v15 >= v17 - v23) {
                goto LABEL_26;
            }
            if (v6 >= 2) {
                int _R0 = smmul(v50 + v49 + v17, 1431655766);
                v29 = _R0 + (_R0 >> 31);
                v28 = v29;
                if (v29 < v50) {
                    v29 = v50;
                }
                if (v28 >= v17) {
                    v29 = v17 - 1;
                }
                v30 = (intptr_t)(v58 - 1);
                do {
                    v31 = *(int *)(uintptr_t)(v30 + 4);
                    v30 += 4;
                } while (v31 <= v29);
                v32 = (int)(4 * (v30 - (intptr_t)v54));
                v55[v6 - 1] = v32;
                if (v32) {
                    v33 = *(int *)v30;
                    v34 = *(int *)v30 - *(int *)(v30 - 4);
                    if (v34) {
                        v35 = 16 * (v33 - v29) / v34;
                    } else {
                        v35 = 8;
                    }
                    v55[v6 - 1] = v32 - v35;
                }
            }
            v14 = 0;
            v55[v6++] = (int)((char *)v18 - (char *)v54) * 4;
            v51 = v17;
            v17 = v15;
            v58 = v18;
LABEL_44:
            v19 = v52;
        } while ((uintptr_t)v18 < v52);
    }
    return v6;
}

/* source: libFsaDecoderCore1.c:64997-65100 */
static unsigned int SmartSymbolRedef0006016145(signed int a1, signed int a2)
{
    signed int v2;
    unsigned int v3;
    unsigned int result;
    signed int v5;
    int v6;
    int v7;
    int v8;
    int v9;
    int v10;
    int v11;
    int v12;
    int v13;
    int v14;
    int v15;
    int v16;
    int v17;
    int v18;
    int v19;
    int v20;
    int *v21;
    int v22;
    int v23;
    int v24;
    int v25;
    int v26;
    int v27;
    int v28;
    int v29;
    int v30;
    int v31;
    int v32;
    int v33;
    int v34;
    unsigned int v35;
    int v36;

    v2 = a1;
    v3 = (unsigned int)((dword_2081D4 << 10) + 2048);
    result = 0;
    if ((signed int)v3 <= (v2 >> 2)) {
        v3 = (unsigned int)(v2 >> 2);
        if ((dword_2081D8 << 10) - 2048 >= (v2 >> 2)) {
            v5 = a2 >> 2;
            if (!dword_2081E0 || ((dword_2081CC << 10) + 2048 <= v5 && (dword_2081D0 << 10) - 2048 >= v5)) {
                const uint8_t *b = (const uint8_t *)dword_2081AC;
                v6 = dword_2081B4 * ((dword_2081C8 & (a2 >> 12)) - 1) + (v2 >> 12);
                v7 = b[v6 - 1];
                v8 = b[v6];
                v9 = b[v6 + 1];
                v10 = b[v6 + 2];
                v11 = (int)(uintptr_t)(b + v6 - 1 + dword_2081B4 - 4);
                v12 = b[v11 + 4 - (int)(uintptr_t)b];
                v11 += 4;
                v13 = v12;
                v14 = v11 + dword_2081B4 - 4;
                v15 = b[v11 + 1 - (int)(uintptr_t)b];
                v16 = b[v11 + 2 - (int)(uintptr_t)b];
                v17 = (v3 >> 1) & 0x1E0;
                v18 = b[v11 + 3 - (int)(uintptr_t)b];
                v19 = b[v14 + 4 - (int)(uintptr_t)b];
                v14 += 4;
                v20 = v14 + dword_2081B4 - 4;
                v21 = (int *)((uint8_t *)SmartSymbolRedef0005ef33 + v17 + 4);
                v22 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v17 + 4);
                v23 = b[v14 + 1 - (int)(uintptr_t)b];
                v24 = b[v14 + 2 - (int)(uintptr_t)b];
                v25 = b[v14 + 3 - (int)(uintptr_t)b];
                v26 = *(v21 + 1);
                v27 = *(v21 + 2);
                v28 = *(v21 + 3);
                v29 = b[v20 + 4 - (int)(uintptr_t)b] * v22 + b[v20 + 5 - (int)(uintptr_t)b] * v26 + b[v20 + 6 - (int)(uintptr_t)b] * v27;
                v30 = v13 * v22 + v15 * v26 + v16 * v27 + v18 * v28;
                v31 = v7 * v22 + v8 * v26 + v9 * v27 + v10 * v28;
                v32 = v19 * v22 + v23 * v26 + v24 * v27;
                v33 = ((unsigned int)v5 >> 1) & 0x1E0;
                v3 = *(unsigned int *)((uint8_t *)SmartSymbolRedef0005ef33 + v33 + 16);
                v34 = *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v33 + 4) * v31
                    + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v33 + 8) * v30
                    + *(int *)((uint8_t *)SmartSymbolRedef0005ef33 + v33 + 12) * (v32 + v25 * v28);
                v35 = 0xFFFFFF;
                v36 = v34 + (int)v3 * (v29 + v28 * b[v20 + 7 - (int)(uintptr_t)b]) + 0x8000;
                result = 0;
                if (v36 < 0xFFFFFF) {
                    v35 = (unsigned int)v36;
                }
                if (v36 > 0) {
                    result = v35 >> 16;
                }
            }
        }
    }
    return result;
}

/* source: src/decompiled_signal/sinc_interpolation.c:1-48 */
static signed int ia_pp_sinc_point_max(signed int a1, signed int a2)
{
    signed int v2;
    signed int v3;
    signed int v4;
    signed int v5;
    signed int v6;
    signed int v7;
    signed int v8;
    signed int v9;
    signed int v10;
    signed int v11;
    signed int v12;
    signed int v13;
    signed int v14;

    v2 = a2;
    v3 = a1;
    v4 = (signed int)SmartSymbolRedef0006016145(a1, a2);
    v5 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v2);
    if (v5 > v4) {
        v4 = v5;
    }
    v6 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v2);
    v7 = v2 - 4096;
    if (v6 > v4) {
        v4 = v6;
    }
    v8 = (signed int)SmartSymbolRedef0006016145(v3, v2 - 4096);
    v9 = v2 + 4096;
    if (v8 > v4) {
        v4 = v8;
    }
    v10 = (signed int)SmartSymbolRedef0006016145(v3, v9);
    if (v10 > v4) {
        v4 = v10;
    }
    v11 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v7);
    if (v11 > v4) {
        v4 = v11;
    }
    v12 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v7);
    if (v12 > v4) {
        v4 = v12;
    }
    v13 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v9);
    if (v13 > v4) {
        v4 = v13;
    }
    v14 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v9);
    if (v14 > v4) {
        v4 = v14;
    }
    return v4;
}

/* source: src/decompiled_signal/sinc_interpolation.c:50-96 */
static signed int ia_pp_sinc_point_min(signed int a1, signed int a2)
{
    signed int v2;
    signed int v3;
    signed int v4;
    signed int v5;
    signed int v6;
    signed int v7;
    signed int v8;
    signed int v9;
    signed int v10;
    signed int v11;
    signed int v12;
    signed int v13;
    signed int v14;

    v2 = a2;
    v3 = a1;
    v4 = (signed int)SmartSymbolRedef0006016145(a1, a2);
    v5 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v2);
    if (v5 < v4) {
        v4 = v5;
    }
    v6 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v2);
    v7 = v2 - 4096;
    if (v6 < v4) {
        v4 = v6;
    }
    v8 = (signed int)SmartSymbolRedef0006016145(v3, v2 - 4096);
    v9 = v2 + 4096;
    if (v8 < v4) {
        v4 = v8;
    }
    v10 = (signed int)SmartSymbolRedef0006016145(v3, v9);
    if (v10 < v4) {
        v4 = v10;
    }
    v11 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v7);
    if (v11 < v4) {
        v4 = v11;
    }
    v12 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v7);
    if (v12 < v4) {
        v4 = v12;
    }
    v13 = (signed int)SmartSymbolRedef0006016145(v3 - 4096, v9);
    if (v13 < v4) {
        v4 = v13;
    }
    v14 = (signed int)SmartSymbolRedef0006016145(v3 + 4096, v9);
    if (v14 < v4) {
        v4 = v14;
    }
    return v4;
}

/* source: src/decompiled_signal/ia_transitions.c:1-84 */
static int ia_transitions(int a1, int a2, int a3, int a4, _DWORD *a5, _DWORD *a6, signed int a7)
{
    int v11;
    int v13;
    int *v14;
    int *v15;
    signed int v23;
    int v24;
    int v25;
    int v26;
    int v27;
    int v30;
    int v31;

    v11 = a4 - a2;
    if (a4 - a2 < 0) {
        v11 = -v11;
    }
    int _R5 = a3 - a1;
    if (_R5 < 0) {
        _R5 = -_R5;
    }
    if (_R5 < v11) {
        _R5 = v11;
    }
    if (_R5 >= 6) {
        v13 = _R5 + 4;
        v14 = (int *)malloc((size_t)8 * (size_t)(_R5 + 4));
        v15 = v14;
        if (v14) {
            /* original ASM block:
               VMOV/VCVT/VSUB/VDIV to compute float start + per-step delta */
            float fR1 = (float)a1;
            float fR2 = (float)a2;
            float fR3 = ((float)a3 - (float)a1) / (float)_R5;
            float v29 = ((float)a4 - (float)a2) / (float)_R5;

            SmartSymbolRedef0006016140(v14, fR1, fR2, fR3, v29, _R5);
            SmartSymbolRedef000601676(v15, _R5, &v30, &v31);
            v23 = SmartSymbolRedef0006016143(v15, (_DWORD *)&v15[v13], v31, v30, _R5, a7);
            *a6 += v23;
            if (v23 >= 3) {
                v24 = (int)(uintptr_t)&v15[_R5];
                v25 = 0;
                v26 = 0;
                do {
                    v27 = v24 + 4 * v26;
                    v26 += 2;
                    v25 = *(int *)(uintptr_t)(v27 + 24) + v25 - *(int *)(uintptr_t)(v27 + 20);
                } while (v26 < v23 - 2);
                *a5 += 100 * v25 / *(&v15[v23 - 1] + v13);
            }
            free(v15);
        }
    }
    return 0;
}

/* source: src/decompiled_signal/dmfi_thresh.c:1-75 (adapted state inputs) */
static int dmfi_thresh_local(const int *signal, int length)
{
    int v18[65];
    memset(v18, 0, sizeof(v18));

    if (length < 2) {
        return 127;
    }

    int *v2 = (int *)malloc((size_t)length * sizeof(int));
    if (!v2) {
        return 127;
    }

    memcpy(v2, signal, (size_t)length * sizeof(int));

    for (int i = 0; i < length - 1; i++) {
        v2[i] = v2[i + 1] - v2[i];
    }

    int v5 = length - 1;
    for (int i = 0; i < length - 1; i++) {
        int v7 = v2[i];
        if (v7 < 0) {
            v7 = -v7;
        }
        --v5;
        ++*(int *)((char *)v18 + (v7 & 0xFFFFFFFC));
    }

    int v6 = v18[0];
    int v9 = 1;
    int v10 = 7 * (length - v6) / 8;
    int v4 = 0;
    int result;
    int v12;
    while (1) {
        result = v18[v9];
        v4 += result;
        if (v10 < v4) {
            break;
        }
        v12 = v9 + 1;
        if (v9++ >= 63) {
            goto LABEL_12;
        }
    }
    v12 = v9;
    result = 4 * v9 + 8;
    if (result >= 127) {
        result = 127;
    }
LABEL_12:
    if (v12 == 64) {
        result = 127;
    }

    free(v2);
    return result;
}

/*
 * Transition finder uses min/max only as coarse anchors.
 * Tighten the working range with DMFI-derived local contrast so thresholds
 * track local signal structure instead of the global extrema.
 */
static void signal_adaptive_thresholds(const int *signal, int len, int *thresh_low, int *thresh_high)
{
    int lo = 0;
    int hi = 0;
    if (len > 0) {
        SmartSymbolRedef000601676((int *)signal, len, &lo, &hi);
    }
    if (len <= 1 || hi <= lo) {
        *thresh_low = lo;
        *thresh_high = hi;
        return;
    }

    int mid = (lo + hi) / 2;
    int dmfi = dmfi_thresh_local(signal, len);
    int half = dmfi / 2;
    if (half < 4) {
        half = 4;
    }
    int max_half = (hi - lo) / 2;
    if (half > max_half) {
        half = max_half;
    }
    if (half < 1) {
        half = 1;
    }

    *thresh_low = mid - half;
    *thresh_high = mid + half;
}

/* source: libFsaDecoderCore1.c:115969-116069 (SmartSymbolRedef0006016184)
 * Two-pass (forward then backward) asymmetric envelope tracker, Q8 fixed point.
 * a3 receives the "upper" envelope (attacks fast / decays slow via shift a5),
 * a4 receives the "lower" envelope (attacks slow / decays fast via shift a6).
 * Called with a5=8, a6=10 from the far-field edge chain (sub_15BA6C). */
static signed int SmartSymbolRedef0006016184(_DWORD *a1, signed int a2, signed int *a3, signed int *a4, char a5, char a6)
{
    signed int v6;
    intptr_t v18;
    intptr_t v19;
    signed int *v7;
    int *v8;
    int *v9;
    signed int v10;
    signed int v11;
    int v12;
    int v13;
    signed int result;
    int v15;
    int v16;
    signed int v17;
    signed int v20;
    int v21;
    int v22;
    signed int v23;
    int v24;
    signed int v25;
    signed int *v26;

    v6 = *a1 << 8;
    *a4 = v6;
    *a3 = v6;
    if (a2 <= 1) {
        *a3 = v6 >> 8;
        result = *a4 >> 8;
        *a4 = result;
        return result;
    }

    v24 = a2 - 1;
    v7 = a3 + 1;
    v8 = a4 + 1;
    v9 = a1 + 1;
    v10 = v6;
    v26 = a4;
    v25 = a2;
    do {
        v11 = v10 - v6;
        v12 = *v9;
        --a2;
        ++v9;
        v13 = v10 - ((v10 - v6) >> a5);
        result = v12 << 8;
        *v8 = v12 << 8;
        if (v13 > v12 << 8) {
            result = v13;
        }
        *v7 = result;
        ++v7;
        v15 = *v8;
        v16 = v6 + (v11 >> a6);
        if (v16 < *v8) {
            *v8 = v16;
            v15 = v6 + (v11 >> a6);
        }
        ++v8;
        v6 = v15;
        v10 = result;
    } while (a2 > 1);

    a3[v24] >>= 8;
    a4[v24] >>= 8;
    v17 = v25;
    if (v25 >= 2) {
        v18 = (intptr_t)(a3 - 2);
        v19 = (intptr_t)(v26 - 2);
        do {
            v20 = *(signed int *)(v18 + 4 * v17);
            v21 = v15 + ((result - v15) >> a6);
            v22 = result - ((result - v15) >> a5);
            if (v22 > v20) {
                *(signed int *)(v18 + 4 * v17) = v22;
                v20 = v22;
            }
            v23 = v20;
            v15 = *(signed int *)(v19 + 4 * v17);
            if (v21 < v15) {
                *(signed int *)(v19 + 4 * v17) = v21;
                v15 = v21;
                v23 = *(signed int *)(v18 + 4 * v17);
            }
            *(signed int *)(v18 + 4 * v17) = v23 >> 8;
            *(signed int *)(v19 + 4 * v17--) >>= 8;
            result = v20;
        } while (v17 > 1);
    }
    return result;
}

/* source: libFsaDecoderCore1.c:116366-116548 (SmartSymbolRedef0006016183b)
 * Walks the signal a1[] tracking alternating local extrema against the
 * adaptive threshold built from the envelope pair (a3=upper, a4=lower from
 * SmartSymbolRedef0006016184). Emits candidate transition indices (signed,
 * sign encodes rising/falling) into a2, returns the candidate count.
 * a5 is unused by the original (kept for ABI fidelity with the call site). */
static signed int SmartSymbolRedef0006016183b(_DWORD *a1, _DWORD *a2, _DWORD *a3, _DWORD *a4, int a5, int a6)
{
    (void)a5;
    _DWORD *v6;
    _DWORD *v7;
    uintptr_t v8;
    signed int v9;
    int *v10;
    int v11;
    int v12;
    int v13;
    signed int v14;
    int v15;
    _DWORD *v16;
    int v17;
    intptr_t v18;
    int v19;
    signed int v20;
    signed int v21;
    int v22;
    int v24;
    signed int v25;
    int v26;
    int v33;
    int v34;
    int v35;
    int v36;
    int v37;
    int v38;
    _DWORD *v40;
    _DWORD *v41;
    _DWORD *v42;
    signed int v43;
    uintptr_t v44;
    intptr_t v45;
    signed int v46;
    int _R8;
    int _R0;

    v6 = a4;
    v7 = a3;
    v41 = a2;
    *a2 = 0;
    v8 = (uintptr_t)&a1[a6 - 1];
    v9 = 1;
    if (v8 < (uintptr_t)a1) {
        return v9;
    }

    v10 = a1 + 1;
    v11 = *a1;
    v12 = 4 * a6 - 4;
    v13 = 1;
    v14 = 1;
    v15 = *a1 - 1;
    v43 = 1;
    v16 = a1;
    v40 = a1;
    v45 = (intptr_t)a1;
    v44 = (uintptr_t)&a1[a6 - 1];
    while (1) {
        v18 = (intptr_t)(v10 - 1);
        if (!(v14 & 1)) {
            break;
        }
        if (v11 > v15) {
            v46 = v14;
            v42 = v6;
            v22 = *v6;
            _R8 = *v7 - *v6;
            v24 = v11 - v22;
            v25 = 8;
            if (_R8 > 8) {
                v25 = _R8;
            }
            v26 = (v24 << 10) / v25;
            if (v26 == 1024) {
                _R0 = 715827883;
                v33 = smmul(_R8, _R0);
                v33 = v33 + (v33 >> 31);
                if (v33 <= 3) {
                    v33 = 3;
                }
                v13 = v11 - v33;
            } else {
                if (v26 < 923) {
                    v36 = 3;
                    if (_R8 / 16 > 3) {
                        v36 = _R8 / 16;
                    }
                } else {
                    v36 = 3;
                    if (_R8 / 8 > 3) {
                        v36 = _R8 / 8;
                    }
                }
                v13 = v11 - v36;
            }
            v45 = (intptr_t)(v10 - 1);
            v18 = (intptr_t)v16;
            v15 = v11;
            v6 = v42;
            v8 = v44;
LABEL_45:
            v14 = v46;
            if ((uintptr_t)v10 > v8) {
                return v43;
            }
            goto LABEL_3;
        }
        if (v12 && v11 >= v13) {
            goto LABEL_24;
        }
        if (v43 >= 2) {
            v41[v43] = v16 - v40;
            v15 = *(v10 - 1);
            ++v43;
        } else {
            v15 = v11;
        }
        ++v14;
        v37 = (*v7 - *v6) / 16;
        v38 = 3;
        if (v37 > 3) {
            v38 = v37;
        }
        v13 = v38 + v15;
        if ((uintptr_t)v10 > v8) {
            return v43;
        }
LABEL_3:
        v12 -= 4;
        v17 = *v10;
        ++v10;
        v11 = v17;
        ++v7;
        ++v6;
        v16 = (_DWORD *)v18;
    }

    if (v11 >= v15) {
        if (!v12 || v11 > v13) {
            ++v14;
            v41[v43] = (int)((v45 - (intptr_t)v40) >> 2);
            v15 = *(v10 - 1);
            ++v43;
            v45 = (intptr_t)(v10 - 1);
            v34 = 3;
            if ((*v7 - *v6) / 16 > 3) {
                v34 = (*v7 - *v6) / 16;
            }
            v13 = v15 - v34;
        }
LABEL_24:
        v18 = (intptr_t)v16;
        if ((uintptr_t)v10 > v8) {
            return v43;
        }
        goto LABEL_3;
    }

    v46 = v14;
    v19 = v12;
    v20 = *v7 - *v6;
    v21 = 8;
    if (v20 > 8) {
        v21 = *v7 - *v6;
    }
    if (((v11 - *v6) << 10) / v21 > 204) {
        v35 = 3;
        if (v20 / 16 > 3) {
            v35 = v20 / 16;
        }
        v13 = v35 + v11;
    } else {
        v13 = v11 + 1;
    }
    v15 = v11;
    v8 = v44;
    v12 = v19;
    goto LABEL_45;
}

/* source: libFsaDecoderCore1.c:118197-118269 (Point_Spread)
 * Extends the two endpoints of a segment {result, a2} (each a {x,y} int
 * pair) outward along the segment direction: result moves away from a2 by
 * ratio a3/a4, a2 moves away from result by ratio a5/a4. Used to lengthen a
 * scan segment past its detected endpoints before 8x-oversampling it
 * (Point_Spread in the far-field edge chain, sub_15BA6C). */
static intptr_t Point_Spread(intptr_t result, intptr_t a2, int a3, unsigned int a4, int a5)
{
    intptr_t v5;
    intptr_t v6;
    int v7;
    signed int v8;
    int v9;
    unsigned int v10;
    unsigned int v11;
    int v12;
    unsigned int v13;
    unsigned int v14;
    int v15;
    unsigned int v16;
    int v17;
    int v18;
    int v19;
    unsigned int v20;
    int v21;
    int v22;
    int v23;
    int v24;
    int v25;

    v5 = a2;
    v6 = result;
    if (a4) {
        v7 = *(_DWORD *)(result + 4);
        v8 = (signed int)a4;
        v23 = *(_DWORD *)a2;
        v25 = *(_DWORD *)result;
        v9 = *(_DWORD *)(a2 + 4) - v7;
        v24 = v9;
        v10 = (unsigned int)(v9 * a3);
        v11 = v10 >> 31;
        if (v10 & 0x80000000u) {
            v10 = -v10;
        }
        if (a4 & 0x80000000u) {
            v8 = -(signed int)a4;
        }
        v12 = (signed int)(v10 + (unsigned int)(v8 >> 1)) / v8;
        if (((v11 & 0xFFu) + (unsigned int)((int)a4 >> 31)) & 1) {
            v12 = -v12;
        }
        *(_DWORD *)(v6 + 4) = v7 - v12;

        v13 = (unsigned int)((v23 - v25) * a3);
        v14 = v13 >> 31;
        if (v13 & 0x80000000u) {
            v13 = -v13;
        }
        v15 = (signed int)(v13 + (unsigned int)(v8 >> 1)) / v8;
        if ((v14 + (unsigned int)((int)a4 >> 31)) & 1) {
            v15 = -v15;
        }
        *(_DWORD *)v6 = v25 - v15;

        v16 = (unsigned int)((v23 - v25) * a5);
        v17 = v24 * a5;
        if (v24 * a5 < 0) {
            v17 = -v17;
        }
        v18 = (v17 + (v8 >> 1)) / v8;
        if ((((unsigned int)(v24 * a5) >> 31) + (unsigned int)((int)a4 >> 31)) & 1) {
            v18 = -v18;
        }
        v19 = *(_DWORD *)v5;
        *(_DWORD *)(v5 + 4) += v18;

        v20 = v16 >> 31;
        if (v16 & 0x80000000u) {
            v16 = -v16;
        }
        v21 = (int)v20 + ((int)a4 >> 31);
        v22 = (signed int)(v16 + (unsigned int)(v8 >> 1)) / v8;
        if (v21 & 1) {
            v22 = -v22;
        }
        result = v22 + v19;
        *(_DWORD *)v5 = (_DWORD)result;
    }
    return result;
}

/* source: src/decompiled_signal/envelope_edge_finder.c:1-66
 * a1/a4/a5 widened from the original `int` to intptr_t: on a 64-bit host the
 * plain-`int` address truncation used by the original decompile silently
 * corrupts pointers > 4GB. This primitive had no caller before this change
 * (dead code), so the widening does not alter any exercised behavior. */
static int vp_iv_edge_finder_envelope_strong_peak(intptr_t a1, _DWORD *a2, int *a3, intptr_t a4, intptr_t a5, int a6, _DWORD *a7, _DWORD *a8)
{
    intptr_t v8;
    _DWORD *v9;
    int v10;
    int v11;
    intptr_t v12;
    int v13;
    int v14;
    int v15;
    signed int v16;
    int v17;
    int v18;
    int v20;
    int v21;
    int *v22;

    v8 = a1;
    v9 = a2;
    v22 = a3;
    v10 = a6 + (a6 & 1);
    v21 = 0;
    v20 = 0;
    if (v10 - 1 >= 3) {
        v11 = v10 - 3;
        v12 = (intptr_t)(a2 + 2);
        v13 = 0;
        v20 = 0;
        v21 = 0;
        do {
            v14 = *(int *)(v12 + 4 * v13);
            v15 = *(int *)(a5 + 4 * v14);
            v16 = *(int *)(a4 + 4 * v14) - v15;
            v17 = *(int *)(v8 + 4 * v14) - v15;
            if (v16 <= 8) {
                v16 = 8;
            }
            v18 = (v17 << 10) / v16;
            if ((v13 + 2) & 1) {
                if (v18 > 700) {
                    *v9 = v14;
                    ++v9;
                    *v22 = v18;
                    ++v22;
                    ++v20;
                }
            } else if (v18 <= 204) {
                *v9 = -v14;
                ++v9;
                *v22 = v18;
                ++v22;
                ++v21;
            }
            ++v13;
        } while (v11 != v13);
    }
    *a7 = v21;
    *a8 = v20;
    return v20 + v21;
}

/* Public wrapper around Point_Spread (far-field edge chain, sub_15BA6C):
 * extends the two endpoints of a horizontal-ish segment outward along its
 * own direction by ratio a3/a4 (at *x0,*y0) and a5/a4 (at *x1,*y1). Ratios
 * mirror the original call site (a3=1, a4=10, a5=1 -> +10% each end). */
void signal_point_spread(int *x0, int *y0, int *x1, int *y1,
                          int num_a3, unsigned int den_a4, int num_a5)
{
    int p0[2] = { *x0, *y0 };
    int p1[2] = { *x1, *y1 };
    Point_Spread((intptr_t)p0, (intptr_t)p1, num_a3, den_a4, num_a5);
    *x0 = p0[0]; *y0 = p0[1];
    *x1 = p1[0]; *y1 = p1[1];
}

/* Public wrapper around SmartSymbolRedef0006016184 (far-field edge chain):
 * two-pass asymmetric envelope tracker. raw[] is the (oversampled) signal,
 * n its length; upper_env[]/lower_env[] (each length n) receive the fast-
 * attack/slow-decay and slow-attack/fast-decay envelope pair. attack_shift/
 * decay_shift mirror the original call site's a5=8, a6=10. */
void signal_far_envelope(const int *raw, int n, int *upper_env, int *lower_env,
                          int attack_shift, int decay_shift)
{
    if (n <= 0) return;
    SmartSymbolRedef0006016184((_DWORD *)raw, n, upper_env, lower_env,
                                (char)attack_shift, (char)decay_shift);
}

int signal_sample(const uint8_t *image, int width, int height,
                  int x0, int y0, int x1, int y1,
                  int *signal, int max_len)
{
    if (x0 < 2) x0 = 2;
    if (x1 > width - 3) x1 = width - 3;
    if (y0 < 2) y0 = 2;
    if (y1 > height - 3) y1 = height - 3;

    int dx = x1 - x0;
    int dy = y1 - y0;
    if (dx < 0) {
        dx = -dx;
    }
    if (dy < 0) {
        dy = -dy;
    }
    int len = dx > dy ? dx : dy;
    if (len < 1) {
        return 0;
    }
    if (len > max_len) {
        len = max_len;
    }

    dword_2081AC = (uintptr_t)image;
    dword_2081B4 = width;
    dword_2081C8 = -1;
    dword_2081CC = 1;
    dword_2081D0 = height - 2;
    dword_2081D4 = 1;
    dword_2081D8 = width - 2;
    dword_2081E0 = 1;

    float fx0 = (float)x0;
    float fy0 = (float)y0;
    float fdx = ((float)x1 - (float)x0) / (float)len;
    float fdy = ((float)y1 - (float)y0) / (float)len;

    SmartSymbolRedef0006016140(signal, fx0, fy0, fdx, fdy, len);
    return len;
}

void signal_envelope(const int *signal, int len, int *thresh_low, int *thresh_high)
{
    signal_adaptive_thresholds(signal, len, thresh_low, thresh_high);
}

int signal_find_edges(const int *signal, int *work_buf, int len,
                      int thresh_low, int thresh_high,
                      int *edges, int max_edges, int flags)
{
    int *buf = work_buf;
    int *tmp = NULL;
    if (!buf) {
        tmp = (int *)malloc((size_t)len * sizeof(int));
        if (!tmp) {
            return 0;
        }
        buf = tmp;
    }

    int n = SmartSymbolRedef0006016143((int *)signal, (_DWORD *)buf, thresh_high, thresh_low, len, flags);
    if (n < 0) {
        n = 0;
    }
    if (n > max_edges) {
        n = max_edges;
    }
    for (int i = 0; i < n; i++) {
        edges[i] = buf[i] / 16;
    }

    free(tmp);
    return n;
}

/* Like signal_find_edges but returns edges in Q4 fixed-point (16x sub-pixel precision).
 * The valley tracker already computes sub-pixel positions — this avoids truncation. */
int signal_find_edges_q4(const int *signal, int *work_buf, int len,
                         int thresh_low, int thresh_high,
                         int *edges_q4, int max_edges, int flags)
{
    int *buf = work_buf;
    int *tmp = NULL;
    if (!buf) {
        tmp = (int *)malloc((size_t)len * sizeof(int));
        if (!tmp) return 0;
        buf = tmp;
    }

    int n = SmartSymbolRedef0006016143((int *)signal, (_DWORD *)buf, thresh_high, thresh_low, len, flags);
    if (n < 0) n = 0;
    if (n > max_edges) n = max_edges;
    for (int i = 0; i < n; i++) {
        edges_q4[i] = buf[i];  /* keep full Q4 precision */
    }

    free(tmp);
    return n;
}

int signal_sinc_max(int a1, int a2)
{
    return ia_pp_sinc_point_max(a1, a2);
}

int signal_sinc_min(int a1, int a2)
{
    return ia_pp_sinc_point_min(a1, a2);
}

static int signal_clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* mirror of sub_2AE40 */
static int signal_max2(int a, int b)
{
    return (a >= b) ? a : b;
}

/* mirror of sub_2AE7C */
static int signal_abs1(int a)
{
    return (a >= 0) ? a : -a;
}

static int signal_denoise_edges(const int *in_edges, int n_in, int *out_edges, int max_out)
{
    if (!in_edges || !out_edges || n_in <= 0 || max_out <= 0) {
        return 0;
    }
    int n = 0;
    int prev = in_edges[0];
    out_edges[n++] = prev;
    for (int i = 1; i < n_in && n < max_out; i++) {
        int e = in_edges[i];
        if (e <= prev) {
            continue;
        }
        if (e - prev <= 0) {
            continue;
        }
        out_edges[n++] = e;
        prev = e;
    }
    return n;
}

static int signal_edge_quality_score(const int *edges, int n)
{
    if (!edges || n < 2) {
        return INT_MIN / 2;
    }
    int tiny = 0;
    int narrow = 0;
    int wide = 0;
    for (int i = 0; i + 1 < n; i++) {
        int g = edges[i + 1] - edges[i];
        if (g <= 0) {
            return INT_MIN / 2;
        }
        if (g <= 1) tiny++;
        else if (g <= 3) narrow++;
        else if (g >= 120) wide++;
    }
    return n * 16 - tiny * 42 - narrow * 8 - wide * 4;
}

static int signal_edge_primitives_score(const int *edges, int n)
{
    if (!edges || n < 10) {
        return -64;
    }

    ScanRuntime runtime = {0};
    ScanEdges scan_edges = {(int *)edges, n};
    ScanContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.runtime = &runtime;
    ctx.edges = &scan_edges;

    int score = 0;
    int samples = 0;
    int last_pat = -1;

    for (int i = 0; i + 8 < n && samples < 64; i += 2) {
        int span = edges[i + 8] - edges[i];
        if (span <= 0) {
            continue;
        }
        int thr = span / 8;
        if (thr < 1) {
            thr = 1;
        }

        int bit_votes = WnBit(&ctx, i, thr)
            + WnBit(&ctx, i + 2, thr)
            + WnBit(&ctx, i + 4, thr)
            + WnBit(&ctx, i + 6, thr);
        int bars = Bars(&ctx, i, 4);
        int pat = WnPat(&ctx, i, 8, thr, thr);

        score += bit_votes * 5;
        score += bars / 6;
        score += __builtin_popcount((unsigned int)pat) * 2;
        if (last_pat >= 0 && pat != last_pat) {
            score += 6;
        }
        last_pat = pat;
        samples++;
    }

    if (samples == 0) {
        return -64;
    }
    return score / samples;
}

/*
 * Conservative impulse conditioning:
 * - edge activity is measured with max(abs(gradL), abs(gradR))
 * - only outliers beyond local activity envelope are clipped
 * This keeps true transitions while suppressing narrow spikes.
 */
static void signal_condition_absmax(const int *src, int len, int *dst)
{
    if (!src || !dst || len <= 0) {
        return;
    }
    if (len <= 2) {
        for (int i = 0; i < len; i++) dst[i] = src[i];
        return;
    }

    dst[0] = src[0];
    dst[len - 1] = src[len - 1];
    for (int i = 1; i + 1 < len; i++) {
        int l = src[i - 1];
        int c = src[i];
        int r = src[i + 1];
        int grad_l = c - l;
        int grad_r = r - c;
        int act = signal_max2(signal_abs1(grad_l), signal_abs1(grad_r));
        int center = (l + c + r) / 3;
        int delta = c - center;
        int ad = signal_abs1(delta);
        int limit = act + 3;
        if (ad > limit) {
            int clip = limit;
            if (delta < 0) clip = -clip;
            dst[i] = center + clip;
        } else {
            dst[i] = c;
        }
    }
}

/*
 * Deeper two-pass conditioning chain used by Code128 only.
 * Mirrors sub_2AE40/sub_2AE7C primitives via signal_max2/signal_abs1.
 */
static void signal_condition_sub_2AEB8(const int *src, int len, int *tmp, int *dst)
{
    if (!src || !tmp || !dst || len <= 0) {
        return;
    }
    if (len <= 4) {
        for (int i = 0; i < len; i++) dst[i] = src[i];
        return;
    }

    tmp[0] = src[0];
    tmp[len - 1] = src[len - 1];
    for (int i = 1; i + 1 < len; i++) {
        int l = src[i - 1];
        int c = src[i];
        int r = src[i + 1];
        int g0 = signal_abs1(c - l);
        int g1 = signal_abs1(r - c);
        int act = signal_max2(g0, g1);

        int g2 = g0;
        int g3 = g1;
        if (i >= 2) g2 = signal_abs1(c - src[i - 2]) / 2;
        if (i + 2 < len) g3 = signal_abs1(src[i + 2] - c) / 2;
        act = signal_max2(act, signal_max2(g2, g3));

        int center = (l + c + r) / 3;
        int delta = c - center;
        int limit = act + 2;
        if (signal_abs1(delta) > limit) {
            tmp[i] = center + ((delta < 0) ? -limit : limit);
        } else {
            tmp[i] = c;
        }
    }

    dst[0] = tmp[0];
    dst[len - 1] = tmp[len - 1];
    for (int i = 1; i + 1 < len; i++) {
        int l = tmp[i - 1];
        int c = tmp[i];
        int r = tmp[i + 1];
        int smooth = (l + (c << 1) + r) >> 2;
        int edge = signal_max2(signal_abs1(c - l), signal_abs1(r - c));
        int delta = c - smooth;
        int limit = edge + 2;
        if (signal_abs1(delta) > limit) {
            dst[i] = smooth + ((delta < 0) ? -limit : limit);
        } else {
            dst[i] = c;
        }
    }
}

/*
 * Low-contrast enhancement for long-distance scans:
 * 1) min-max normalize to [0,255]
 * 2) remove low-frequency trend (high-pass)
 * 3) light denoise smoothing
 * 4) local adaptive gain from local activity
 * 5) blend with normalized signal to keep structure stable
 */
static void signal_preprocess_enhanced(const int *src, int len, int *norm, int *tmp, int *dst)
{
    int smin = src[0];
    int smax = src[0];
    for (int i = 1; i < len; i++) {
        if (src[i] < smin) smin = src[i];
        if (src[i] > smax) smax = src[i];
    }

    int range = smax - smin;
    if (range < 1) {
        for (int i = 0; i < len; i++) {
            norm[i] = signal_clampi(src[i], 0, 255);
            dst[i] = norm[i];
        }
        return;
    }

    for (int i = 0; i < len; i++) {
        norm[i] = (src[i] - smin) * 255 / range;
    }

    int w_trend = len / 48;
    if (w_trend < 9) w_trend = 9;
    if (w_trend > 63) w_trend = 63;
    if ((w_trend & 1) == 0) w_trend++;
    int r_trend = w_trend / 2;
    for (int i = 0; i < len; i++) {
        int l = i - r_trend;
        int r = i + r_trend;
        if (l < 0) l = 0;
        if (r >= len) r = len - 1;
        int sum = 0;
        int cnt = r - l + 1;
        for (int j = l; j <= r; j++) {
            sum += norm[j];
        }
        tmp[i] = sum / cnt;
    }

    for (int i = 0; i < len; i++) {
        dst[i] = signal_clampi(norm[i] - tmp[i] + 128, 0, 255);
    }

    int w_smooth = 5;
    int r_smooth = w_smooth / 2;
    for (int i = 0; i < len; i++) {
        int l = i - r_smooth;
        int r = i + r_smooth;
        if (l < 0) l = 0;
        if (r >= len) r = len - 1;
        int sum = 0;
        int cnt = r - l + 1;
        for (int j = l; j <= r; j++) {
            sum += dst[j];
        }
        tmp[i] = sum / cnt;
    }

    int w_gain = len / 36;
    if (w_gain < 11) w_gain = 11;
    if (w_gain > 91) w_gain = 91;
    if ((w_gain & 1) == 0) w_gain++;
    int r_gain = w_gain / 2;
    for (int i = 0; i < len; i++) {
        int l = i - r_gain;
        int r = i + r_gain;
        if (l < 0) l = 0;
        if (r >= len) r = len - 1;
        int cnt = r - l + 1;
        int activity = 0;
        for (int j = l; j <= r; j++) {
            int d = tmp[j] - 128;
            if (d < 0) d = -d;
            activity += d;
        }
        activity /= cnt;
        int gain = (48 * 256) / (activity + 12);
        gain = signal_clampi(gain, 160, 896);
        int v = 128 + ((tmp[i] - 128) * gain >> 8);
        dst[i] = signal_clampi(v, 0, 255);
    }

    for (int i = 0; i < len; i++) {
        dst[i] = (3 * dst[i] + norm[i]) / 4;
    }
}

static int signal_transitions_internal(const uint8_t *image, int width, int height,
                                       int x0, int y0, int x1, int y1,
                                       int *edges, int max_edges,
                                       int code128_deep_chain)
{
    if (x0 < 2) x0 = 2;
    if (x1 > width - 3) x1 = width - 3;
    if (y0 < 2) y0 = 2;
    if (y1 > height - 3) y1 = height - 3;

    int dx = x1 - x0;
    int dy = y1 - y0;
    if (dx < 0) {
        dx = -dx;
    }
    if (dy < 0) {
        dy = -dy;
    }
    int len = dx > dy ? dx : dy;
    if (len < 6) {
        return 0;
    }
    if (len > MAX_SIGNAL_LEN) {
        len = MAX_SIGNAL_LEN;
    }

    int *v15 = (int *)malloc((size_t)8 * (size_t)(len + 4));
    if (!v15) {
        return 0;
    }

    int signal_len = signal_sample(image, width, height, x0, y0, x1, y1, v15, len);
    if (signal_len <= 0) {
        free(v15);
        return 0;
    }

    int v30 = 0;
    int v31 = 0;
    signal_envelope(v15, signal_len, &v30, &v31);

    int v13 = signal_len + 4;
    int *best_edges = (int *)malloc((size_t)max_edges * sizeof(int));
    if (!best_edges) {
        free(v15);
        return 0;
    }

    /* Try both polarities (flags=1: start peaks, flags=0: start valleys) */
    int best_n = 0;
    int best_score = INT_MIN / 2;
    {
        int *cand = (int *)malloc((size_t)max_edges * sizeof(int));
        if (cand) {
            for (int fl = 1; fl >= 0; fl--) {
                int n = signal_find_edges(v15, &v15[v13], signal_len, v30, v31, edges, max_edges, fl);
                int nc = signal_denoise_edges(edges, n, cand, max_edges);
                int sc = signal_edge_quality_score(cand, nc)
                    + signal_edge_primitives_score(cand, nc);
                if (sc > best_score || (sc == best_score && nc > best_n)) {
                    if (nc > 0) {
                        memcpy(best_edges, cand, (size_t)nc * sizeof(int));
                        best_n = nc;
                        best_score = sc;
                    }
                }
            }
            free(cand);
        }
    }

    if (best_n > 0) {
        memcpy(edges, best_edges, (size_t)best_n * sizeof(int));
    }

    /*
     * Keep near-distance behavior stable: only enhance when baseline edges are
     * sparse; dense edge maps keep the original chain untouched.
     */
    if (best_n >= 56) {
        free(best_edges);
        free(v15);
        return best_n;
    }

    int *norm = (int *)malloc((size_t)signal_len * sizeof(int));
    int *tmp = (int *)malloc((size_t)signal_len * sizeof(int));
    int *enh = (int *)malloc((size_t)signal_len * sizeof(int));
    int *cond = (int *)malloc((size_t)signal_len * sizeof(int));
    int *deep = (int *)malloc((size_t)signal_len * sizeof(int));
    int *cand_edges = (int *)malloc((size_t)max_edges * sizeof(int));
    int *cand_clean = (int *)malloc((size_t)max_edges * sizeof(int));
    if (!norm || !tmp || !enh || !cond || !deep || !cand_edges || !cand_clean) {
        free(cand_clean);
        free(cand_edges);
        free(deep);
        free(cond);
        free(enh);
        free(tmp);
        free(norm);
        free(best_edges);
        free(v15);
        return best_n;
    }

    signal_preprocess_enhanced(v15, signal_len, norm, tmp, enh);

    int lo = 0;
    int hi = 0;
    signal_envelope(enh, signal_len, &lo, &hi);
    int mid = (lo + hi) / 2;
    int half = (hi - lo) / 2;
    if (half < 4) half = 4;

    static const int half_scales[] = {100, 130, 160, 200, 240};
    static const int mid_shifts[] = {0, -6, 6};
    for (size_t si = 0; si < sizeof(half_scales) / sizeof(half_scales[0]); si++) {
        for (size_t mi = 0; mi < sizeof(mid_shifts) / sizeof(mid_shifts[0]); mi++) {
            int h = half * half_scales[si] / 100;
            int m = mid + mid_shifts[mi];
            int tlo = signal_clampi(m - h, 0, 255);
            int thi = signal_clampi(m + h, 0, 255);
            if (thi <= tlo) {
                continue;
            }
            int n = signal_find_edges(enh, &v15[v13], signal_len, tlo, thi, cand_edges, max_edges, 1);
            int n_clean = signal_denoise_edges(cand_edges, n, cand_clean, max_edges);
            int score = signal_edge_quality_score(cand_clean, n_clean)
                + signal_edge_primitives_score(cand_clean, n_clean);
            if (score > best_score || (score == best_score && n_clean > best_n)) {
                if (n_clean > 0) {
                    memcpy(best_edges, cand_clean, (size_t)n_clean * sizeof(int));
                    best_n = n_clean;
                    best_score = score;
                }
            }
        }
    }

    signal_condition_absmax(v15, signal_len, cond);
    signal_envelope(cond, signal_len, &lo, &hi);
    for (int fl = 1; fl >= 0; fl--) {
        int n = signal_find_edges(cond, &v15[v13], signal_len, lo, hi, cand_edges, max_edges, fl);
        int n_clean = signal_denoise_edges(cand_edges, n, cand_clean, max_edges);
        int score = signal_edge_quality_score(cand_clean, n_clean)
            + signal_edge_primitives_score(cand_clean, n_clean);
        int strong_gain = (n_clean >= best_n + 3) && (score >= best_score + 24);
        if (strong_gain || score > best_score + 72) {
            if (n_clean > 0) {
                memcpy(best_edges, cand_clean, (size_t)n_clean * sizeof(int));
                best_n = n_clean;
                best_score = score;
            }
        }
    }

    if (code128_deep_chain) {
        signal_condition_sub_2AEB8(v15, signal_len, tmp, deep);
        signal_envelope(deep, signal_len, &lo, &hi);
        int mid = (lo + hi) / 2;
        int half = (hi - lo) / 2;
        if (half < 4) half = 4;
        static const int deep_half_scales[] = {90, 115, 140, 170};
        static const int deep_mid_shifts[] = {0, -8, 8};
        for (size_t si = 0; si < sizeof(deep_half_scales) / sizeof(deep_half_scales[0]); si++) {
            for (size_t mi = 0; mi < sizeof(deep_mid_shifts) / sizeof(deep_mid_shifts[0]); mi++) {
                int h = half * deep_half_scales[si] / 100;
                int m = mid + deep_mid_shifts[mi];
                int tlo = signal_clampi(m - h, 0, 255);
                int thi = signal_clampi(m + h, 0, 255);
                if (thi <= tlo) {
                    continue;
                }
                for (int fl = 1; fl >= 0; fl--) {
                    int n = signal_find_edges(deep, &v15[v13], signal_len, tlo, thi, cand_edges, max_edges, fl);
                    int n_clean = signal_denoise_edges(cand_edges, n, cand_clean, max_edges);
                    int score = signal_edge_quality_score(cand_clean, n_clean)
                        + signal_edge_primitives_score(cand_clean, n_clean);
                    int better_quality = score >= best_score + 12;
                    int better_recall = (n_clean >= best_n + 1) && (score >= best_score - 36);
                    int unlock_dense = (best_n < 56) && (n_clean >= 56) && (score >= best_score - 48);
                    if ((better_quality || better_recall || unlock_dense) && n_clean > 0) {
                        memcpy(best_edges, cand_clean, (size_t)n_clean * sizeof(int));
                        best_n = n_clean;
                        best_score = score;
                    }
                }
            }
        }
    }

    if (best_n > 0) {
        memcpy(edges, best_edges, (size_t)best_n * sizeof(int));
    }
    free(cand_clean);
    free(cand_edges);
    free(deep);
    free(cond);
    free(enh);
    free(tmp);
    free(norm);
    free(best_edges);
    free(v15);
    return best_n;
}

int signal_transitions(const uint8_t *image, int width, int height,
                       int x0, int y0, int x1, int y1,
                       int *edges, int max_edges)
{
    return signal_transitions_internal(image, width, height, x0, y0, x1, y1,
                                       edges, max_edges, 0);
}

int signal_transitions_code128(const uint8_t *image, int width, int height,
                               int x0, int y0, int x1, int y1,
                               int *edges, int max_edges)
{
    return signal_transitions_internal(image, width, height, x0, y0, x1, y1,
                                       edges, max_edges, 1);
}
