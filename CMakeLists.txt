cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/port/components"
    "${CMAKE_CURRENT_LIST_DIR}/port/standalone_idf_test/main")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p4scan_ov9281)
