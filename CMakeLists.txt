cmake_minimum_required(VERSION 3.20)

project(
    AeroForge
    VERSION 0.1.0
    DESCRIPTION "6DOF aerospace simulation, guidance, navigation, and control platform"
    LANGUAGES CXX
)

add_library(AeroForgeCore
    src/math/Vector3.cpp
    src/math/Matrix.cpp
    src/math/Quaternion.cpp
    src/geometry/Transform.cpp
    src/geometry/FrameTransform.cpp
)

target_compile_features(AeroForgeCore
    PUBLIC
        cxx_std_20
)

target_include_directories(AeroForgeCore
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(AeroForgeCore
        PRIVATE
            /W4
            /permissive-
            /Zc:__cplusplus
    )
else()
    target_compile_options(AeroForgeCore
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

add_executable(AeroForge
    src/main.cpp
)

target_link_libraries(AeroForge
    PRIVATE
        AeroForgeCore
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

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()