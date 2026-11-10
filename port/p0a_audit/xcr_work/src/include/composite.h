#ifndef COMPOSITE_H
#define COMPOSITE_H

#include "barcode.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Restored control tags and output command IDs from:
 * libFsaDecoderCore1.c fmdclsymbol0009 (L233000-L237000 focus region).
 */
#define COMPOSITE_TAG_CC_ENABLE          0x1A026001u
#define COMPOSITE_TAG_TLC39_ENABLE       0x1A017001u
#define COMPOSITE_TAG_TLC39_A            0x1A023001u
#define COMPOSITE_TAG_TLC39_B            0x1A024001u
#define COMPOSITE_TAG_TLC39_C            0x1A025001u
#define COMPOSITE_TAG_TLC39_STATE        0x1A030001u

#define COMPOSITE_CMD_ENABLE_MASK        679875585u
#define COMPOSITE_CMD_TLC39_A            679870981u
#define COMPOSITE_CMD_TLC39_B            679872257u
#define COMPOSITE_CMD_TLC39_C            679872258u
#define COMPOSITE_CMD_TLC39_STATE        679871237u

#define COMPOSITE_PENDING_CAP 31

typedef enum {
    COMPOSITE_FORMAT_NONE = 0,
    COMPOSITE_FORMAT_CC_A = 1,
    COMPOSITE_FORMAT_CC_B = 2,
    COMPOSITE_FORMAT_CC_C = 3,
    COMPOSITE_FORMAT_TLC39 = 4
} CompositeFormat;

typedef struct {
    uint32_t tag;
    int value;
    int used;
} CompositePendingEntry;

typedef struct {
    int composite_enabled;
    int tlc39_enabled;
    int defer_tlc39;

    CompositePendingEntry pending[COMPOSITE_PENDING_CAP];
} CompositeState;

typedef int (*CompositeEmitFn)(void *ctx, uint32_t command, int value);
typedef int (*CompositeQueryFn)(void *ctx, uint32_t command, int *value_out);

typedef int (*CompositeLinearDecodeFn)(const Image *img, BarcodeResult *out, void *ctx);
typedef int (*CompositeComponentDecodeFn)(
    const Image *img,
    CompositeFormat format,
    BarcodeResult *out,
    void *ctx);

typedef struct {
    CompositeFormat format;
    BarcodeResult linear;
    BarcodeResult component;
    BarcodeResult merged;
} CompositeDecodeResult;

void composite_state_init(CompositeState *st);

uint32_t composite_compose_enable_mask(const CompositeState *st);

/*
 * Handles composite/TLC39 control tags restored from the dispatcher region.
 * Returns:
 *  1 on handled success,
 *  0 on unknown/unhandled tag,
 * -1 on handled but failed (emit/queue full/invalid args).
 */
int composite_handle_control_tag(
    CompositeState *st,
    uint32_t tag,
    int value,
    CompositeEmitFn emit,
    void *emit_ctx);

/* Flush deferred TLC39 control tags in queued order. */
int composite_flush_pending(CompositeState *st, CompositeEmitFn emit, void *emit_ctx);

/* Seed deferred TLC39 state by querying current command values. */
int composite_seed_pending(CompositeState *st, CompositeQueryFn query, void *query_ctx);

/*
 * Lightweight format framework:
 * - TLC39 is explicit when requested.
 * - Non-TLC formats are bucketed as CC-A/B/C by component size.
 */
CompositeFormat composite_detect_format(int prefer_tlc39, int component_rows, int component_cols);

/*
 * Composite decode orchestration framework:
 * 1) decode linear payload (Code128/GS1/Code39 path supplied by caller)
 * 2) decode composite component (CC-A/B/C or TLC39 component supplied by caller)
 * 3) merge into a single output payload
 */
int composite_decode(
    const Image *img,
    CompositeFormat format_hint,
    CompositeLinearDecodeFn linear_decode,
    void *linear_ctx,
    CompositeComponentDecodeFn component_decode,
    void *component_ctx,
    CompositeDecodeResult *out);

#ifdef __cplusplus
}
#endif

#endif /* COMPOSITE_H */
