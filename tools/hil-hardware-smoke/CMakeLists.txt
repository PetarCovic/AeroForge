cmake_minimum_required(VERSION 3.20)

project(
    AeroForgeHilHardwareSmokeTest
    LANGUAGES CXX
)

if(NOT WIN32)
    message(FATAL_ERROR
        "This hardware smoke test requires Windows."
    )
endif()

add_executable(
    AeroForgeHilHardwareSmokeTest
    main.cpp
)

target_compile_features(
    AeroForgeHilHardwareSmokeTest
    PRIVATE cxx_std_20
)

target_compile_definitions(
    AeroForgeHilHardwareSmokeTest
    PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
)

set_target_properties(
    AeroForgeHilHardwareSmokeTest
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
            "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG
            "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE
            "${CMAKE_BINARY_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO
            "${CMAKE_BINARY_DIR}/bin"
)

if(MSVC)
    target_compile_options(
        AeroForgeHilHardwareSmokeTest
        PRIVATE /W4 /permissive-
    )
endif()
