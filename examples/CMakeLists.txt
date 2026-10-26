
#
# Ballistic projectile example
#
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

#
# 6DOF flight recording example
#
add_executable(
    AeroForge6DoFRecordingExample
    rocket_6dof_flight.cpp
)

target_link_libraries(
    AeroForge6DoFRecordingExample
    PRIVATE
        AeroForgeCore
)

target_compile_features(
    AeroForge6DoFRecordingExample
    PRIVATE
        cxx_std_20
)

#
# Compiler warnings
#
foreach(example_target IN ITEMS
    AeroForgeBallisticExample
    AeroForge6DoFRecordingExample
)
    if(MSVC)
        target_compile_options(
            ${example_target}
            PRIVATE
                /W4
                /permissive-
                /Zc:__cplusplus
        )
    else()
        target_compile_options(
            ${example_target}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
        )
    endif()
endforeach()
