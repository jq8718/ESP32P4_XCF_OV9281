/* ESP32-P4: 16 independent unsigned pixels per vector. The C caller
 * supplies 16-byte-aligned rows and handles the left/right borders. */
    .text
    .p2align 2
    .global xcr_dm_sample_row2_pie
    .type xcr_dm_sample_row2_pie, @function
xcr_dm_sample_row2_pie:
    /* One aligned source row, aligned output, pixel count (multiple of 16). */
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.vunzip.8 q0, q1
    esp.vst.128.ip q0, a1, 16
    addi a2, a2, -16
    bnez a2, 1b
    ret
    .size xcr_dm_sample_row2_pie, .-xcr_dm_sample_row2_pie

    .p2align 2
    .global xcr_dm_vertical_pie
    .type xcr_dm_vertical_pie, @function
xcr_dm_vertical_pie:
    /* top, center, bottom, lo, hi, padded_width */
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a2, 16
    esp.vmin.u8 q3, q0, q1
    esp.vmax.u8 q4, q0, q1
    esp.vmin.u8 q3, q3, q2
    esp.vmax.u8 q4, q4, q2
    esp.vst.128.ip q3, a3, 16
    esp.vst.128.ip q4, a4, 16
    addi a5, a5, -16
    bnez a5, 1b
    ret
    .size xcr_dm_vertical_pie, .-xcr_dm_vertical_pie

    .p2align 2
    .global xcr_dm_sample2_pie
    .type xcr_dm_sample2_pie, @function
xcr_dm_sample2_pie:
    /* Three source rows, three aligned outputs, pixel count / row.
     * VUNZIP keeps the even bytes of two vectors in q0. */
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.vunzip.8 q0, q1
    esp.vst.128.ip q0, a3, 16
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a1, 16
    esp.vunzip.8 q0, q1
    esp.vst.128.ip q0, a4, 16
    esp.vld.128.ip q0, a2, 16
    esp.vld.128.ip q1, a2, 16
    esp.vunzip.8 q0, q1
    esp.vst.128.ip q0, a5, 16
    addi a6, a6, -16
    bnez a6, 1b
    ret
    .size xcr_dm_sample2_pie, .-xcr_dm_sample2_pie

    .p2align 2
    .global xcr_dm_classify_pie
    .type xcr_dm_classify_pie, @function
xcr_dm_classify_pie:
    /* lo, hi, center, dst, length (multiple of 16), two constant vectors.
     * The lo/hi pointers start one vector before the first center. */
    esp.vld.128.ip q6, a5, 16  /* Min contrast minus one. */
    esp.vld.128.ip q7, a5, 0   /* Dark threshold bias: 0 or 6. */
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.srci.2q q1, q0, 10   /* left taps: center - 5 */
    addi a0, a0, -16
    esp.vld.128.ip q1, a0, 16
    esp.vmin.u8 q0, q0, q1
    esp.vld.128.ip q2, a0, -16
    esp.srci.2q q2, q1, 4    /* right taps: center + 5 */
    esp.vmin.u8 q0, q0, q1

    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.srci.2q q2, q1, 10
    addi a1, a1, -16
    esp.vld.128.ip q2, a1, 16
    esp.vmax.u8 q1, q1, q2
    esp.vld.128.ip q3, a1, -16
    esp.srci.2q q3, q2, 4
    esp.vmax.u8 q1, q1, q2

    /* Black iff h-l > 8 and (h-g) > (g-l)+bias. All differences
     * are nonnegative. Clamp before subtracting the bias. */
    esp.vld.128.ip q2, a2, 16
    esp.vsub.u8 q3, q1, q0
    esp.vsub.u8 q4, q2, q0
    esp.vsub.u8 q5, q1, q2
    esp.vmax.u8 q5, q5, q7
    esp.vsub.u8 q5, q5, q7
    esp.vcmp.gt.u8 q3, q3, q6
    esp.vcmp.gt.u8 q5, q5, q4
    esp.andq q3, q3, q5
    esp.vst.128.ip q3, a3, 16
    addi a4, a4, -16
    bnez a4, 1b
    ret
    .size xcr_dm_classify_pie, .-xcr_dm_classify_pie
