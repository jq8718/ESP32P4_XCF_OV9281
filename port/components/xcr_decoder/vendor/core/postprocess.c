#include "postprocess.h"

#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

static int map_symbol_hint_to_check(int symbol_hint, char *out)
{
    switch (symbol_hint) {
        case 0:  *out = '6'; return 1;
        case 2:  *out = '9'; return 1;
        case 3:  *out = '5'; return 1;
        case 6:  *out = '8'; return 1;
        case 7:  *out = '7'; return 1;
        case 9:  *out = '4'; return 1;
        case 14: *out = '3'; return 1;
        case 15: *out = '2'; return 1;
        case 17: *out = '1'; return 1;
        case 28: *out = '0'; return 1;
        default: return 0;
    }
}

void postprocess_reverse(char *s, size_t len)
{
    for (size_t i = 0; i < len / 2; i++) {
        char t = s[i];
        s[i] = s[len - 1 - i];
        s[len - 1 - i] = t;
    }
}

static int postprocess_delete_at(char *s, size_t *len, size_t pos)
{
    if (!s || !len || pos >= *len) return 0;
    for (size_t i = pos; i + 1 < *len; i++) s[i] = s[i + 1];
    (*len)--;
    s[*len] = '\0';
    return 1;
}

static int postprocess_insert_at(char *s, size_t *len, size_t cap, char ch, size_t pos)
{
    if (!s || !len || *len >= cap || pos > *len) return 0;
    for (size_t i = *len; i > pos; i--) s[i] = s[i - 1];
    s[pos] = ch;
    (*len)++;
    s[*len] = '\0';
    return 1;
}

int postprocess_insert_prefix(char *s, size_t *len, size_t cap, const char *prefix)
{
    size_t n;

    if (!s || !len || !prefix) return 0;
    n = strlen(prefix);
    if (*len + n >= cap) return 0;

    for (size_t i = *len + n; i > n; i--) s[i - 1] = s[i - n - 1];
    for (size_t i = 0; i < n; i++) s[i] = prefix[i];
    *len += n;
    s[*len] = '\0';
    return 1;
}

int postprocess_append_addenda(char *s, size_t *len, size_t cap, const char *addenda, size_t addenda_len)
{
    if (!s || !len || !addenda) return 0;
    if (addenda_len != 2 && addenda_len != 5) return 0;
    if (*len + 1 + addenda_len >= cap) return 0;

    s[*len] = ' ';
    for (size_t i = 0; i < addenda_len; i++) s[*len + 1 + i] = addenda[i];
    *len += 1 + addenda_len;
    s[*len] = '\0';
    return 1;
}

/* SmartSymbolRedef0006016276B checksum form used at 293689. */
static int smartsymbol_mod10_remainder(const char *digits, size_t len)
{
    int sum = 0;

    if (!digits || len == 0) return -1;
    for (size_t i = 0; i < len; i++) {
        int d;
        int weight;

        if (digits[i] < '0' || digits[i] > '9') return -1;
        d = digits[i] - '0';
        weight = (((len - i) & 1) == 0) ? 3 : 1;
        sum += d * weight;
    }
    return sum % 10;
}

char postprocess_isbn10_check_digit(const char nine_digits[9])
{
    /* Equivalent weighted form at 293866..293875. */
    int sum = 0;
    static const char isbn_chks[] = "0123456789X";

    for (int i = 0; i < 9; i++) {
        if (nine_digits[i] < '0' || nine_digits[i] > '9') return '?';
        sum += (10 - i) * (nine_digits[i] - '0');
    }
    return isbn_chks[(11 - (sum % 11)) % 11];
}

int postprocess_upce_to_upca(const char *upce, char *out, size_t out_sz)
{
    char in[9];
    size_t n;
    char ns;
    char c1, c2, c3, c4, c5, c6;
    char chk;

    if (!upce || !out || out_sz < 13) return 0;
    n = strlen(upce);
    if (n != 7 && n != 8) return 0;

    if (n == 7) {
        in[0] = '0';
        memcpy(in + 1, upce, 7);
        in[8] = '\0';
    } else {
        memcpy(in, upce, 8);
        in[8] = '\0';
    }

    for (int i = 0; i < 8; i++) {
        if (in[i] < '0' || in[i] > '9') return 0;
    }

    ns = in[0];
    c1 = in[1]; c2 = in[2]; c3 = in[3]; c4 = in[4]; c5 = in[5]; c6 = in[6];
    chk = in[7];

    out[0] = ns;
    switch (c6) {
        case '0':
        case '1':
        case '2':
            out[1] = c1; out[2] = c2; out[3] = c6;
            out[4] = '0'; out[5] = '0'; out[6] = '0'; out[7] = '0';
            out[8] = c3; out[9] = c4; out[10] = c5;
            break;
        case '3':
            out[1] = c1; out[2] = c2; out[3] = c3;
            out[4] = '0'; out[5] = '0'; out[6] = '0'; out[7] = '0'; out[8] = '0';
            out[9] = c4; out[10] = c5;
            break;
        case '4':
            out[1] = c1; out[2] = c2; out[3] = c3; out[4] = c4;
            out[5] = '0'; out[6] = '0'; out[7] = '0'; out[8] = '0'; out[9] = '0';
            out[10] = c5;
            break;
        default:
            out[1] = c1; out[2] = c2; out[3] = c3; out[4] = c4; out[5] = c5;
            out[6] = '0'; out[7] = '0'; out[8] = '0'; out[9] = '0';
            out[10] = c6;
            break;
    }

    out[11] = chk;
    out[12] = '\0';
    return 1;
}

static void apply_isbn_postaddenda_rules(PostprocessState *st)
{
    size_t base_len = st->length;

    if (st->addenda_len < 5) {
        st->length = 10;
        st->data[10] = '\0';
        return;
    }
    if (base_len < 5) return;

    /* Mirrors 293878..293909 price/addenda handling. */
    {
        size_t idx = base_len - 5;
        char lead = st->data[idx];

        if (lead == '5') {
            if (base_len < 4 ||
                st->data[base_len - 4] != '9' ||
                st->data[base_len - 3] != '9' ||
                st->data[base_len - 2] != '9' ||
                st->data[base_len - 1] != '9') {
                st->data[idx] = '0';
            } else {
                st->length = 10;
                st->data[10] = '\0';
            }
            return;
        }

        if (lead >= '1' && lead <= '4') return;

        st->length = 10;
        st->data[10] = '\0';
    }
}

int fmwlsymbol003e_postprocess(PostprocessState *st)
{
    char check;
    int special = 0;

    if (!st) return 0;
    if (st->length >= POSTPROCESS_MAX_LEN) return 0;
    st->data[st->length] = '\0';

    /* Source: libFsaDecoderCore1.c:293587-293651 (Reverse + orientation gate). */
    if (st->reverse_before) {
        postprocess_reverse(st->data, st->length);
        if (st->reverse_guard) return 0;
    }

    /* Source: 293656-293699 (symbol hint -> inserted check + mod10 gate). */
    if (!map_symbol_hint_to_check(st->symbol_hint, &check)) return 0;
    if (!postprocess_insert_at(st->data, &st->length, POSTPROCESS_MAX_LEN - 1, check, 0)) return 0;
    if (smartsymbol_mod10_remainder(st->data, st->length) != 0) return 0;

    st->code_id[0] = 'E';
    st->code_id[1] = '0';
    st->code_id[2] = '\0';

    /* Source: 293700-293790 (zero-NS and nonzero-NS UPC/EAN handling + GS1 mode). */
    if (st->data[0] == '0' && st->cfg_248_enable_zero_ns) {
        postprocess_delete_at(st->data, &st->length, 0);

        if (st->cfg_277_special_addenda && st->length > 0 && st->data[0] == '5') special = 1;

        if (st->cfg_279_gs1_mode == 1 || st->cfg_279_gs1_mode == 2) {
            if (!postprocess_insert_prefix(st->data, &st->length, POSTPROCESS_MAX_LEN - 1, "0100")) return 0;
        } else if (st->cfg_279_gs1_mode == 3) {
            st->code_id[0] = 'C';
            st->code_id[1] = '1';
        } else if (st->cfg_279_gs1_mode == 0) {
            if (!st->cfg_249_keep_len_mode0 && st->length > 0) st->length--;
            if (!st->cfg_250_keep_first_mode0) postprocess_delete_at(st->data, &st->length, 0);
            st->data[st->length] = '\0';
        }
    } else {
        if (!st->cfg_264_enable_nonzero_ns) return 0;

        if (st->cfg_277_special_addenda && st->length > 1) {
            if ((st->data[0] == '0' && st->data[1] == '5') ||
                (st->data[0] == '9' && st->data[1] == '9')) {
                special = 1;
            }
        }

        if (st->cfg_279_gs1_mode == 1 || st->cfg_279_gs1_mode == 2) {
            if (!postprocess_insert_prefix(st->data, &st->length, POSTPROCESS_MAX_LEN - 1, "010")) return 0;
            st->code_id[0] = 'C';
            st->code_id[1] = '1';
        } else if (st->cfg_279_gs1_mode == 3) {
            st->code_id[0] = 'C';
            st->code_id[1] = '1';
        } else if (st->cfg_279_gs1_mode == 0 && !st->cfg_265_trim_nonzero_mode0 && st->length > 0) {
            st->length--;
            st->data[st->length] = '\0';
        }
    }

    /* Source: 293810-293834 (TryAddend path). Here addenda is passed in pre-decoded form. */
    if (st->addenda_len == 2 || st->addenda_len == 5) {
        if (!postprocess_append_addenda(st->data, &st->length, POSTPROCESS_MAX_LEN - 1, st->addenda, st->addenda_len)) {
            return 0;
        }
    }

    /* Source: 293846-293853 (centering gate bypass when special addenda fired). */
    if (!special && !st->centered_ok) return 0;

    /* Source: 293854-293910 (EAN-13 978/979 -> ISBN-10 + supplemental handling). */
    if (st->cfg_270_enable_isbn && st->length >= 10 &&
        st->data[0] == '9' && st->data[1] == '7' && (st->data[2] == '8' || st->data[2] == '9')) {
        char chk;

        postprocess_delete_at(st->data, &st->length, 0);
        postprocess_delete_at(st->data, &st->length, 0);
        postprocess_delete_at(st->data, &st->length, 0);
        if (st->cfg_265_trim_nonzero_mode0 && st->length > 9) postprocess_delete_at(st->data, &st->length, 9);

        if (st->length < 9) return 0;
        chk = postprocess_isbn10_check_digit(st->data);
        if (chk == '?') return 0;
        if (!postprocess_insert_at(st->data, &st->length, POSTPROCESS_MAX_LEN - 1, chk, 9)) return 0;

        apply_isbn_postaddenda_rules(st);
        st->code_id[0] = 'E';
        st->code_id[1] = '0';
    }

    st->status_code = 20;
    st->data[st->length] = '\0';
    return 1;
}
