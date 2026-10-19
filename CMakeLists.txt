add_library(modules INTERFACE)
target_link_libraries(modules INTERFACE devices)

file(GLOB _module_include_dirs CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/*/*/include"
)
target_include_directories(modules INTERFACE
    ${_module_include_dirs}
    "${PROJECT_SOURCE_DIR}/configs/include"
)

file(GLOB _module_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*/*/src/*.cpp"
)
# Only the selected IMU service owns acquisition and the shared attitude topic.
list(REMOVE_ITEM _module_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/src/bmi088_service.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/src/bmi088_heater.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/imu/dmimu/src/dmimu_service.cpp"
)
target_sources(modules INTERFACE ${_module_sources})

if(USER_AHRS_DEVICE STREQUAL "bmi088")
    target_sources(modules INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/src/bmi088_service.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088/src/bmi088_heater.cpp"
    )
    target_link_libraries(modules INTERFACE dsp)
elseif(USER_AHRS_DEVICE STREQUAL "dmimu")
    target_sources(modules INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/imu/dmimu/src/dmimu_service.cpp")
endif()
