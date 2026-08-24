include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

add_executable(
    AeroForgeTests
    math/Vector3Tests.cpp
    math/MatrixTests.cpp
    math/QuaternionTests.cpp
    geometry/TransformTests.cpp
    geometry/FrameTransformTests.cpp
    integration/MathGeometryIntegrationTests.cpp
)

target_compile_features(
    AeroForgeTests
    PRIVATE
        cxx_std_20
)

if(MSVC)
    target_compile_options(
        AeroForgeTests
        PRIVATE
            /W4
            /permissive-
            /Zc:__cplusplus
    )
else()
    target_compile_options(
        AeroForgeTests
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

target_link_libraries(
    AeroForgeTests
    PRIVATE
        AeroForgeCore
        GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(AeroForgeTests)