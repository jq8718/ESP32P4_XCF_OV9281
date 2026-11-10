#include "composite.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int command_for_tag(uint32_t tag, uint32_t *out_command)
{
    uint32_t cmd = 0;

    switch (tag) {
    case COMPOSITE_TAG_TLC39_A:
        cmd = COMPOSITE_CMD_TLC39_A;
        break;
    case COMPOSITE_TAG_TLC39_B:
        cmd = COMPOSITE_CMD_TLC39_B;
        break;
    case COMPOSITE_TAG_TLC39_C:
        cmd = COMPOSITE_CMD_TLC39_C;
        break;
    case COMPOSITE_TAG_TLC39_STATE:
        cmd = COMPOSITE_CMD_TLC39_STATE;
        break;
    default:
        return 0;
    }

    if (out_command) {
        *out_command = cmd;
    }
    return 1;
}

static int queue_set(CompositeState *st, uint32_t tag, int value)
{
    int i;
    int free_idx = -1;

    if (!st) {
        return 0;
    }

    for (i = 0; i < COMPOSITE_PENDING_CAP; ++i) {
        if (st->pending[i].used && st->pending[i].tag == tag) {
            st->pending[i].value = value;
            return 1;
        }
        if (!st->pending[i].used && free_idx < 0) {
            free_idx = i;
        }
    }

    if (free_idx < 0) {
        return 0;
    }

    st->pending[free_idx].used = 1;
    st->pending[free_idx].tag = tag;
    st->pending[free_idx].value = value;
    return 1;
}

void composite_state_init(CompositeState *st)
{
    if (!st) {
        return;
    }
    memset(st, 0, sizeof(*st));
}

uint32_t composite_compose_enable_mask(const CompositeState *st)
{
    int composite_on;
    int tlc39_on;

    if (!st) {
        return 0u;
    }

    composite_on = (st->composite_enabled != 0) ? 1 : 0;
    tlc39_on = (st->tlc39_enabled != 0) ? 1 : 0;

    /* Decompiled anchor: a3 = CompositeEnabled | 2 * TLC39CompositeEnabled */
    return (uint32_t)(composite_on | (2 * tlc39_on));
}

int composite_handle_control_tag(
    CompositeState *st,
    uint32_t tag,
    int value,
    CompositeEmitFn emit,
    void *emit_ctx)
{
    uint32_t cmd = 0;

    if (!st) {
        return -1;
    }

    if (tag == COMPOSITE_TAG_CC_ENABLE) {
        st->composite_enabled = value;
        if (!emit) {
            return 1;
        }
        return emit(emit_ctx, COMPOSITE_CMD_ENABLE_MASK, (int)composite_compose_enable_mask(st)) ? 1 : -1;
    }
    if (tag == COMPOSITE_TAG_TLC39_ENABLE) {
        st->tlc39_enabled = value;
        if (!emit) {
            return 1;
        }
        return emit(emit_ctx, COMPOSITE_CMD_ENABLE_MASK, (int)composite_compose_enable_mask(st)) ? 1 : -1;
    }

    if (!command_for_tag(tag, &cmd)) {
        return 0;
    }

    if (st->defer_tlc39) {
        return queue_set(st, tag, value) ? 1 : -1;
    }

    if (!emit) {
        return -1;
    }
    return emit(emit_ctx, cmd, value) ? 1 : -1;
}

int composite_flush_pending(CompositeState *st, CompositeEmitFn emit, void *emit_ctx)
{
    int i;

    if (!st || !emit) {
        return -1;
    }

    for (i = 0; i < COMPOSITE_PENDING_CAP; ++i) {
        uint32_t cmd = 0;
        if (!st->pending[i].used) {
            continue;
        }
        if (!command_for_tag(st->pending[i].tag, &cmd)) {
            st->pending[i].used = 0;
            continue;
        }
        if (!emit(emit_ctx, cmd, st->pending[i].value)) {
            return -1;
        }
        st->pending[i].used = 0;
    }
    return 1;
}

int composite_seed_pending(CompositeState *st, CompositeQueryFn query, void *query_ctx)
{
    static const uint32_t tags[] = {
        COMPOSITE_TAG_TLC39_STATE,
        COMPOSITE_TAG_TLC39_A,
        COMPOSITE_TAG_TLC39_B,
        COMPOSITE_TAG_TLC39_C
    };
    int i;

    if (!st || !query) {
        return -1;
    }

    for (i = 0; i < (int)(sizeof(tags) / sizeof(tags[0])); ++i) {
        int value = 0;
        uint32_t cmd = 0;
        if (!command_for_tag(tags[i], &cmd)) {
            return -1;
        }
        if (!query(query_ctx, cmd, &value)) {
            return -1;
        }
        if (!queue_set(st, tags[i], value)) {
            return -1;
        }
    }
    return 1;
}

CompositeFormat composite_detect_format(int prefer_tlc39, int component_rows, int component_cols)
{
    if (prefer_tlc39) {
        return COMPOSITE_FORMAT_TLC39;
    }

    if (component_rows <= 4 && component_cols <= 30) {
        return COMPOSITE_FORMAT_CC_A;
    }
    if (component_rows <= 20 && component_cols <= 120) {
        return COMPOSITE_FORMAT_CC_B;
    }
    return COMPOSITE_FORMAT_CC_C;
}

static int merge_payload(
    CompositeFormat fmt,
    const BarcodeResult *linear,
    const BarcodeResult *component,
    BarcodeResult *merged)
{
    /* GS1 Composite (CC-A/CC-B/CC-C): the linear symbol's element string and
     * the 2D component's element string are concatenated directly with no
     * separator (verified against the reference decoder's plain-text
     * output for the composite sample set). TLC39 keeps its own "+"
     * separator between the Code39 and Micro PDF417 halves. */
    const char *sep = "";
    int n;

    if (!linear || !component || !merged) {
        return 0;
    }

    if (fmt == COMPOSITE_FORMAT_TLC39) {
        sep = "+";
    }

    n = snprintf(merged->data, sizeof(merged->data), "%s%s%s", linear->data, sep, component->data);
    if (n <= 0 || n >= (int)sizeof(merged->data)) {
        return 0;
    }

    merged->length = n;
    merged->confidence = (linear->confidence + component->confidence) / 2;
    return 1;
}

int composite_decode(
    const Image *img,
    CompositeFormat format_hint,
    CompositeLinearDecodeFn linear_decode,
    void *linear_ctx,
    CompositeComponentDecodeFn component_decode,
    void *component_ctx,
    CompositeDecodeResult *out)
{
    CompositeFormat fmt;
    int linear_ok;
    int component_ok;

    if (!img || !linear_decode || !component_decode || !out) {
        return 0;
    }

    memset(out, 0, sizeof(*out));
    fmt = (format_hint == COMPOSITE_FORMAT_NONE)
        ? COMPOSITE_FORMAT_CC_B
        : format_hint;
    out->format = fmt;

    /* component_decode (MicroPDF417/PDF417) is a fast, highly-selective
     * finder-style probe (milliseconds even on a non-matching render), while
     * linear_decode can fall back to an expensive from-scratch unhinted
     * full-image search when no pre-found linear result is available to
     * reuse. Trying component first lets non-composite images (no 2D
     * component present at all) fail out in milliseconds instead of paying
     * the full linear search cost first. A genuine composite success always
     * requires both linear_ok and component_ok > 0, so this reordering does
     * not change the success set -- only the failure-path cost. */
    component_ok = component_decode(img, fmt, &out->component, component_ctx);
    if (component_ok <= 0) {
        return 0;
    }

    linear_ok = linear_decode(img, &out->linear, linear_ctx);
    if (linear_ok <= 0) {
        return 0;
    }

    if (!merge_payload(fmt, &out->linear, &out->component, &out->merged)) {
        return 0;
    }

    return 1;
}
