#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include <stddef.h>

#define POSTPROCESS_MAX_LEN 64

typedef struct {
    char data[POSTPROCESS_MAX_LEN];
    size_t length;

    /* Input metadata equivalent to fmwlsymbol003e controls. */
    int symbol_hint;              /* maps to switch(v4) @ 293656 */
    int reverse_before;           /* byte +20292 */
    int reverse_guard;            /* byte +20293 when reverse_before != 0 */
    int centered_ok;              /* j_Centered result */

    /* Decoder configuration bytes read around +248..+279. */
    int cfg_248_enable_zero_ns;
    int cfg_264_enable_nonzero_ns;
    int cfg_249_keep_len_mode0;
    int cfg_250_keep_first_mode0;
    int cfg_265_trim_nonzero_mode0;
    int cfg_270_enable_isbn;
    int cfg_277_special_addenda;
    int cfg_279_gs1_mode;         /* 0..3 */

    /* Optional decoded supplemental digits from TryAddend path. */
    char addenda[8];              /* supports 2 or 5 digits */
    size_t addenda_len;

    /* Outputs */
    char code_id[3];              /* "E0" or "C1" */
    int status_code;              /* set to 20 on success path */
} PostprocessState;

/* Restored fmwlsymbol003e-style postprocess workflow. */
int fmwlsymbol003e_postprocess(PostprocessState *st);

/* Focused helpers used by tests and other decoders. */
void postprocess_reverse(char *s, size_t len);
int postprocess_upce_to_upca(const char *upce, char *out, size_t out_sz);
char postprocess_isbn10_check_digit(const char nine_digits[9]);
int postprocess_insert_prefix(char *s, size_t *len, size_t cap, const char *prefix);
int postprocess_append_addenda(char *s, size_t *len, size_t cap, const char *addenda, size_t addenda_len);

#endif
