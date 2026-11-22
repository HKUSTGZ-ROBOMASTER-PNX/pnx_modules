add_library(modules INTERFACE)
target_link_libraries(modules INTERFACE devices dsp)

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
target_sources(modules INTERFACE ${_module_sources})
