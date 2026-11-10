#ifndef POST_VALIDATE_H
#define POST_VALIDATE_H

#include "barcode.h"

#include <stdint.h>

enum {
    POST_VALIDATE_SYM_UPC_EAN = 1u << 0,
    POST_VALIDATE_SYM_CODABAR = 1u << 1,
    POST_VALIDATE_SYM_RSS_GS1 = 1u << 2,
    POST_VALIDATE_SYM_I2OF5   = 1u << 3,
    POST_VALIDATE_SYM_CODE93  = 1u << 4,
};

#define POST_VALIDATE_MASK_DEFAULT \
    (POST_VALIDATE_SYM_UPC_EAN | POST_VALIDATE_SYM_CODABAR | \
     POST_VALIDATE_SYM_RSS_GS1 | POST_VALIDATE_SYM_I2OF5 | POST_VALIDATE_SYM_CODE93)

typedef struct {
    int enabled;
    uint32_t symbology_mask;
    int deep_cross_sym_enabled;
    uint32_t deep_cross_sym_mask;
    int deep_cross_sym_min_conf_gap;
} PostValidateConfig;

void post_validate_config_init_default(PostValidateConfig *cfg);
int post_validate_results(
    const PostValidateConfig *cfg,
    BarcodeResult *results,
    int count);

#endif
