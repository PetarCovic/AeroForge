add_executable(
    AeroForgeBallisticExample
    ballistic_projectile.cpp
)

target_link_libraries(
    AeroForgeBallisticExample
    PRIVATE
    AeroForgeCore
)

target_compile_features(
    AeroForgeBallisticExample
    PRIVATE
    cxx_std_20
)

if(MSVC)
    target_compile_options(
        AeroForgeBallisticExample
        PRIVATE
        /W4
        /permissive-
        /Zc:__cplusplus
    )
else()
    target_compile_options(
        AeroForgeBallisticExample
        PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
endif()