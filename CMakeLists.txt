cmake_minimum_required(VERSION 3.20)

project(
    AeroForge
    VERSION 0.1.0
    DESCRIPTION "6DOF aerospace simulation, guidance, navigation, and control platform"
    LANGUAGES CXX
)

add_executable(AeroForge
    src/main.cpp
)

target_compile_features(AeroForge
    PRIVATE
        cxx_std_20
)

if(MSVC)
    target_compile_options(AeroForge
        PRIVATE
            /W4
            /permissive-
            /Zc:__cplusplus
    )
else()
    target_compile_options(AeroForge
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()