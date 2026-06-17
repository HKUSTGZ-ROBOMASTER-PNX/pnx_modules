cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED PNX_CONFIGS_DIR)
    set(PNX_CONFIGS_DIR ${CMAKE_SOURCE_DIR}/pnx_configs)
endif()

add_library(pnx_modules STATIC
    ahrs/src/ahrs.cpp
    remoter/src/dr16_remoter.cpp
    remoter/src/vt03_remoter.cpp
    referee/src/referee.cpp
    ui/src/ui.cpp
)

target_include_directories(pnx_modules PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/messages/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs/include
    ${CMAKE_CURRENT_SOURCE_DIR}/remoter/include
    ${CMAKE_CURRENT_SOURCE_DIR}/referee/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/include
    ${PNX_CONFIGS_DIR}
)

target_link_libraries(pnx_modules PUBLIC
    pnx_os
    pnx_libraries
    pnx_bsp
    pnx_devices
    pnx_platform
)

add_subdirectory(usb)
