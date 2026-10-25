add_executable(AeroForgeBenchmarks
    main.cpp
    performance/MathBenchmarks.cpp
    performance/IntegratorBenchmarks.cpp
    performance/SixDoFBenchmarks.cpp
    performance/GncBenchmarks.cpp
    performance/TelemetryBenchmarks.cpp
    performance/MonteCarloBenchmarks.cpp
    performance/RealtimeBenchmarks.cpp
)

target_link_libraries(AeroForgeBenchmarks
    PRIVATE
        AeroForgeCore
)

target_compile_features(AeroForgeBenchmarks
    PRIVATE
        cxx_std_20
)

if(MSVC)
    target_compile_options(AeroForgeBenchmarks
        PRIVATE
            /W4
            /permissive-
            /Zc:__cplusplus
    )
else()
    target_compile_options(AeroForgeBenchmarks
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

set_target_properties(AeroForgeBenchmarks
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG
            "${CMAKE_BINARY_DIR}/benchmarks/Debug"

        RUNTIME_OUTPUT_DIRECTORY_RELEASE
            "${CMAKE_BINARY_DIR}/benchmarks/Release"

        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO
            "${CMAKE_BINARY_DIR}/benchmarks/RelWithDebInfo"

        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL
            "${CMAKE_BINARY_DIR}/benchmarks/MinSizeRel"
)