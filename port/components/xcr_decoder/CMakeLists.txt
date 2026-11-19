set(XCR_DECODER_SRCS
    "src/xcr_decoder.c"
    "src/xcr_dmtx.c"
    "src/xcr_port.c"
    "src/xcr_qr_geometry.c"
    "vendor/core/envelope.c"
    "vendor/core/postprocess.c"
    "vendor/core/scan_dispatcher.c"
    "vendor/core/scan_engine.c"
    "vendor/core/scan_primitives.c"
    "vendor/core/signal.c"
    "vendor/symbologies/codablock_f.c"
    "vendor/symbologies/code128.c"
    "vendor/symbologies/upc_ean.c"
    "vendor/symbologies/datamatrix.c"
    "vendor/symbologies/qrcode.c"
    "vendor/qr_quirc.c"
    "vendor/quirc/decode.c"
    "vendor/quirc/identify.c"
    "vendor/quirc/quirc.c"
    "vendor/quirc/version_db.c"
)

set(XCR_DM_BODY_SRCS
    "src/xcr_dm_body.cpp"
    "vendor/zxing_dm/BitMatrix.cpp"
    "vendor/zxing_dm/GridSampler.cpp"
    "vendor/zxing_dm/LocalGrid.cpp"
    "vendor/zxing_dm/PerspectiveTransform.cpp"
    "vendor/zxing_dm/WhiteRectDetector.cpp"
    "vendor/zxing_dm/datamatrix/DMDetector.cpp"
    "vendor/zxing_dm/datamatrix/DMVersion.cpp")
list(APPEND XCR_DECODER_SRCS ${XCR_DM_BODY_SRCS})
if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND XCR_DECODER_SRCS "src/xcr_dm_body_pie.S")
endif()

if(P4SCAN_LOCAL_FILTER_TEST)
    if(NOT P4SCAN_LOCAL_FILTER_VARIANT)
        set(P4SCAN_LOCAL_FILTER_VARIANT gaussian5)
    endif()
    if(NOT P4SCAN_LOCAL_FILTER_VARIANT MATCHES "^(gaussian5|finder_only)$")
        message(FATAL_ERROR "Unsupported local filter test variant")
    endif()
    list(REMOVE_ITEM XCR_DECODER_SRCS "src/xcr_dmtx.c")
    list(APPEND XCR_DECODER_SRCS "../../../test/dm_local_filter/${P4SCAN_LOCAL_FILTER_VARIANT}.c")
    set_source_files_properties("../../../test/dm_local_filter/${P4SCAN_LOCAL_FILTER_VARIANT}.c" PROPERTIES COMPILE_OPTIONS "-O3")
endif()

if(CONFIG_XCR_DATAMATRIX_LIBDMTX)
    list(APPEND XCR_DECODER_SRCS "vendor/libdmtx/dmtx.c")
endif()

idf_component_register(
    SRCS ${XCR_DECODER_SRCS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "vendor/include" "vendor/quirc" "vendor/libdmtx" "vendor/zxing_dm" "../../p0a_audit/xcr_work/src/include"
    PRIV_REQUIRES esp_timer freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -fno-strict-aliasing
    -fstack-usage
    -Wno-unused-function
    -Wno-unused-variable
)
set_source_files_properties(${XCR_DM_BODY_SRCS} PROPERTIES
    COMPILE_DEFINITIONS ZXING_INTERNAL
    COMPILE_OPTIONS "-std=gnu++20;-O3;-fexceptions;-fsingle-precision-constant")
if(NOT P4SCAN_LOCAL_FILTER_TEST)
    set_source_files_properties("src/xcr_dmtx.c" PROPERTIES COMPILE_DEFINITIONS XCR_DMTX_BODY_RECOVERY=1)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    QUIRC_FLOAT_TYPE=float
    QUIRC_USE_TGMATH
)
set_source_files_properties(
    "vendor/symbologies/datamatrix.c"
    PROPERTIES COMPILE_DEFINITIONS "XCR_DATAMATRIX_P4=1;XCR_PORT_REMAP_ALLOC=1"
                  COMPILE_OPTIONS "-O3;-Wno-error=unused-function;-Wno-error=unused-variable"
)
set_source_files_properties(
    "vendor/qr_quirc.c"
    PROPERTIES COMPILE_OPTIONS "-O3"
)
set_source_files_properties("src/xcr_qr_geometry.c" PROPERTIES COMPILE_OPTIONS "-O3")
set_source_files_properties("src/xcr_dmtx.c" PROPERTIES COMPILE_OPTIONS "-O3")
set_source_files_properties(
    "vendor/quirc/decode.c"
    "vendor/quirc/identify.c"
    "vendor/quirc/quirc.c"
    "vendor/quirc/version_db.c"
    PROPERTIES COMPILE_OPTIONS "-O3;-ffast-math"
)
set_source_files_properties(
    "vendor/core/signal.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-error=maybe-uninitialized;-Wno-error=parentheses"
)
set_source_files_properties(
    "vendor/symbologies/codablock_f.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-error=maybe-uninitialized"
)
set_source_files_properties(
    "vendor/symbologies/qrcode.c"
    PROPERTIES COMPILE_OPTIONS "-O3;-ffast-math;-Wno-error=maybe-uninitialized"
)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
set_source_files_properties("vendor/libdmtx/dmtx.c" PROPERTIES
    COMPILE_OPTIONS "-O3;-Wno-error=unused-value;-Wno-error=unused-but-set-variable;-Wno-error=unused-but-set-parameter;-Wno-error=misleading-indentation;-Wno-error=maybe-uninitialized;-Wno-error=return-type"
    COMPILE_DEFINITIONS "HAVE_SYS_TIME_H=1;HAVE_GETTIMEOFDAY=1;VERSION=\"0.7.8\"")
