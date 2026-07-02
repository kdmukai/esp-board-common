cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED BOARD)
    set(BOARD "waveshare_p4_lcd43")
endif()
message(STATUS "Building for board: ${BOARD}")

# esp-board-common component (two levels up)
# esp-camera-pipeline and its sub-components (cam_pipeline_entropy, ...)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    "${CMAKE_CURRENT_SOURCE_DIR}/../../components"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp-camera-pipeline"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../components/esp-camera-pipeline/components"
)

set(BOARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/${BOARD}")
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;${BOARD_DIR}/sdkconfig.defaults")
if(LANDSCAPE)
    set(SDKCONFIG_DEFAULTS "${SDKCONFIG_DEFAULTS};sdkconfig.landscape")
    message(STATUS "Landscape orientation enabled")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(entropy_test)
