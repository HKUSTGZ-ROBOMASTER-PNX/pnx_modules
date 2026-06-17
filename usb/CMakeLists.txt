cmake_minimum_required(VERSION 3.22)

add_library(pnx_usb STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_gateway.c
)

target_include_directories(pnx_usb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pnx_usb PUBLIC
    pnx_os
    pnx_libraries
    pnx_platform
)

target_include_directories(pnx_usb PRIVATE
    ${CMAKE_SOURCE_DIR}/pnx_os/internal/msg_engine
    ${CMAKE_SOURCE_DIR}/pnx_os/internal/msg_engine/app
    ${CMAKE_SOURCE_DIR}/pnx_os/internal/msg_engine/core
    ${CMAKE_SOURCE_DIR}/pnx_os/internal/msg_engine/comp
)

target_compile_features(pnx_usb PUBLIC c_std_11)
