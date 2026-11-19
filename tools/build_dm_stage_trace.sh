#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
out=docs/decode_stages
variant=${XCR_DM_SPLIT_TEST:-0}
objdir=build/dm_stage_trace
binary=$out/dm_stage_trace
if [[ $variant == 1 ]]; then
    objdir=build/dm_stage_trace_split
    binary=$out/dm_stage_trace_split
fi
sanitize=()
if [[ ${XCR_DM_SANITIZE:-0} == 1 ]]; then
    objdir=build/dm_stage_trace_asan
    binary=$objdir/dm_stage_trace
    sanitize=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
fi
mkdir -p "$out" "$objdir"
common=(-O2 -std=c11 -fno-strict-aliasing -DCONFIG_XCR_DATAMATRIX_LIBDMTX=0 -DXCR_DM_TRACE_STAGES=1
        -Iport/components/xcr_decoder/include
        -Iport/components/xcr_decoder/vendor/include
        -Iport/components/xcr_decoder/vendor/quirc)
common+=("${sanitize[@]}")
if [[ $variant == 1 ]]; then common+=(-DXCR_DM_SPLIT_TEST=1); fi
zxing=port/components/xcr_decoder/vendor/zxing_dm
body=()
for source in port/components/xcr_decoder/src/xcr_dm_body.cpp \
    "$zxing"/{BitMatrix,GridSampler,LocalGrid,PerspectiveTransform,WhiteRectDetector}.cpp \
    "$zxing"/datamatrix/{DMDetector,DMVersion}.cpp; do
    object="$objdir/$(basename "${source%.cpp}").o"
    g++ -O3 "${sanitize[@]}" -std=c++20 -fsingle-precision-constant -DZXING_INTERNAL -I"$zxing" -Iport/components/xcr_decoder/include \
        -Iport/components/xcr_decoder/vendor/include -c "$source" -o "$object"
    body+=("$object")
done
gcc "${common[@]}" -c tools/dm_stage_trace.c -o "$objdir/trace_main.o"
gcc "${common[@]}" -DXCR_DMTX_BODY_RECOVERY=1 -c port/components/xcr_decoder/src/xcr_dmtx.c -o "$objdir/trace_marker.o"
gcc "${common[@]}" -c port/components/xcr_decoder/src/xcr_port.c -o "$objdir/trace_port.o"
gcc "${common[@]}" -DXCR_DATAMATRIX_P4=1 -DXCR_PORT_REMAP_ALLOC=1 \
    -c port/components/xcr_decoder/vendor/symbologies/datamatrix.c -o "$objdir/trace_core.o"
g++ "${sanitize[@]}" "$objdir/trace_main.o" "$objdir/trace_marker.o" "$objdir/trace_port.o" "$objdir/trace_core.o" "${body[@]}" \
    -lm -o "$binary"
if [[ ${XCR_DM_SANITIZE:-0} != 1 ]]; then
    gcc "${common[@]}" tools/dm_grid10_probe.c "$objdir/trace_core.o" "$objdir/trace_port.o" \
        -lm -o build/dm_grid10_probe
fi
