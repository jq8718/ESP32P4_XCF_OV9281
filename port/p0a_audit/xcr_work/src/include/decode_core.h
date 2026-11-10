#ifndef DECODE_CORE_H
#define DECODE_CORE_H

#include "barcode.h"
#include "post_validate.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Restored enable block layout for *(ctx+116)+offset consumed by
 * SmartSymbolRedef0001c3 in libFsaDecoderCore1.
 */
typedef struct {
    int32_t reserved_0_64[17];
    int32_t off_68;
    int32_t off_72;
    int32_t off_76;
    int32_t off_80;
    int32_t off_84;
    int32_t off_88;
    int32_t reserved_92_100[3];
    int32_t off_104;
    int32_t off_108;
    int32_t off_112;
    int32_t off_116;
    int32_t off_120;
    int32_t off_124;
    int32_t off_128;
    int32_t off_132;
    int32_t off_136;
    int32_t off_140;
    int32_t off_144;
    int32_t off_148;
    int32_t off_152;
    int32_t off_156;
    int32_t off_160;
    int32_t off_164;
    int32_t reserved_168_188[6];
    int32_t off_192;
    int32_t off_196;
    int32_t off_200;
} DecodeEnableFlags;

_Static_assert(offsetof(DecodeEnableFlags, off_68) == 68, "off_68 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_72) == 72, "off_72 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_76) == 76, "off_76 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_80) == 80, "off_80 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_84) == 84, "off_84 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_88) == 88, "off_88 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_104) == 104, "off_104 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_108) == 108, "off_108 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_112) == 112, "off_112 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_116) == 116, "off_116 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_120) == 120, "off_120 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_124) == 124, "off_124 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_128) == 128, "off_128 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_132) == 132, "off_132 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_136) == 136, "off_136 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_140) == 140, "off_140 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_144) == 144, "off_144 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_148) == 148, "off_148 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_152) == 152, "off_152 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_156) == 156, "off_156 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_160) == 160, "off_160 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_164) == 164, "off_164 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_192) == 192, "off_192 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_196) == 196, "off_196 layout mismatch");
_Static_assert(offsetof(DecodeEnableFlags, off_200) == 200, "off_200 layout mismatch");

typedef struct {
    uint8_t reserved_0_115[116];
    uint32_t enable_block_ptr32; /* ctx+116 in 32-bit ARM */
    int32_t searching_flag;
    int32_t mode_flag;
} DecodeContext;

_Static_assert(offsetof(DecodeContext, enable_block_ptr32) == 116, "ctx+116 layout mismatch");

typedef struct {
    DecodeEnableFlags enable;
    int enable_qr;
    int enable_aztec;
    int enable_datamatrix; /* inferred from FsaDecoderEnableSymbol(38) / 0x1A02C001 */
    int enable_pdf417;     /* inferred from FsaDecoderEnableSymbol(39) / 0x1A100001 */
    int enable_maxicode;   /* inferred from FsaDecoderEnableSymbol(48) / 0x1A02B001 */
    int enable_koreapost;  /* inferred from FsaDecoderEnableSymbol(49) / 0x1C036001 */
    int searching_flag;
    int mode_flag;
    PostValidateConfig post_validate;
    int scheduler_multi_pass; /* 1=layered scheduler, 0=legacy single-flow */
    int decode_timeout_ms;    /* <=0: use internal default */
    int image_budget_ms;      /* <=0: use internal default */
} DecodeConfig;

enum {
    DECODE_SCHED_LAYER_LOW = 0,
    DECODE_SCHED_LAYER_MEDIUM = 1,
    DECODE_SCHED_LAYER_HIGH = 2,
    DECODE_SCHED_LAYER_COUNT = 3
};

enum {
    DECODE_2D_ENTRY_LOW = 0,
    DECODE_2D_ENTRY_HIGH = 1,
    DECODE_2D_ENTRY_COMPOSITE = 2,
    DECODE_2D_ENTRY_COUNT = 3
};

typedef struct {
    uint64_t entered;
    uint64_t hits;
    uint64_t elapsed_ms;
} DecodeSchedulerLayerStats;

typedef struct {
    uint64_t total_images;
    DecodeSchedulerLayerStats layers[DECODE_SCHED_LAYER_COUNT];
} DecodeSchedulerStats;

typedef struct {
    uint64_t qr_attempts[DECODE_2D_ENTRY_COUNT];
    uint64_t aztec_attempts[DECODE_2D_ENTRY_COUNT];
    uint64_t datamatrix_attempts[DECODE_2D_ENTRY_COUNT];
    uint64_t pdf417_attempts[DECODE_2D_ENTRY_COUNT];
    uint64_t maxicode_attempts[DECODE_2D_ENTRY_COUNT];
    uint64_t qr_deep_locator_hits[DECODE_2D_ENTRY_COUNT];
    uint64_t qr_deep_ec_hits[DECODE_2D_ENTRY_COUNT];
    uint64_t qr_deep_payload_hits[DECODE_2D_ENTRY_COUNT];
    uint64_t aztec_deep_locator_hits[DECODE_2D_ENTRY_COUNT];
    uint64_t aztec_deep_ec_hits[DECODE_2D_ENTRY_COUNT];
    uint64_t aztec_deep_payload_hits[DECODE_2D_ENTRY_COUNT];
} Decode2DEntryStats;

void decode_config_init_default(DecodeConfig *cfg);
const char *barcode_symbology_name(int symbology);
const char *decode_scheduler_layer_name(int layer);
const char *decode_2d_entry_name(int source);
void decode_scheduler_stats_reset(void);
void decode_scheduler_stats_get(DecodeSchedulerStats *out);
void decode_2d_entry_stats_reset(void);
void decode_2d_entry_stats_get(Decode2DEntryStats *out);

int decode_image(const Image *img, DecodeConfig *cfg, BarcodeResult *results, int max_results);

#ifdef __cplusplus
}
#endif

#endif
