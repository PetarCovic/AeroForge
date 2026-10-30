# Physical HIL is supported only on Windows hosts.
if(WIN32)
    target_sources(AeroForgeCore PRIVATE
        ${PROJECT_SOURCE_DIR}/src/hil/HilHardwareRun.cpp
    )

    add_executable(AeroForgeHilHardwareBenchmarks
        ${PROJECT_SOURCE_DIR}/benchmarks/hil/HilHardwareBenchmarks.cpp
    )
    target_compile_features(AeroForgeHilHardwareBenchmarks PRIVATE cxx_std_20)
    target_link_libraries(AeroForgeHilHardwareBenchmarks PRIVATE AeroForgeCore)

    if(BUILD_TESTING)
        target_sources(AeroForgeTests PRIVATE
            ${PROJECT_SOURCE_DIR}/tests/hil/HilHardwareRunTests.cpp
        )
    endif()
endif()
