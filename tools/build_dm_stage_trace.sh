#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
out=docs/decode_stages
objdir=build/dm_stage_trace
mkdir -p "$out" "$objdir"
common=(-O2 -std=c11 -fno-strict-aliasing -DCONFIG_XCR_DATAMATRIX_LIBDMTX=0 -DXCR_DM_TRACE_STAGES=1
        -Iport/components/xcr_decoder/include
        -Iport/components/xcr_decoder/vendor/include
        -Iport/components/xcr_decoder/vendor/quirc)
gcc "${common[@]}" -c tools/dm_stage_trace.c -o "$objdir/trace_main.o"
gcc "${common[@]}" -c port/components/xcr_decoder/src/xcr_dmtx.c -o "$objdir/trace_marker.o"
gcc "${common[@]}" -c port/components/xcr_decoder/src/xcr_port.c -o "$objdir/trace_port.o"
gcc "${common[@]}" -DXCR_DATAMATRIX_P4=1 -DXCR_PORT_REMAP_ALLOC=1 \
    -c port/components/xcr_decoder/vendor/symbologies/datamatrix.c -o "$objdir/trace_core.o"
gcc "$objdir/trace_main.o" "$objdir/trace_marker.o" "$objdir/trace_port.o" "$objdir/trace_core.o" \
    -lm -o "$out/dm_stage_trace"
