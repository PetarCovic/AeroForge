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
    simulation/StateTests.cpp
    simulation/EulerIntegratorTests.cpp
    simulation/RK4IntegratorTests.cpp
    simulation/SimulationClockTests.cpp
    simulation/SimulationRunnerTests.cpp
    dynamics/TranslationalStateTests.cpp
    dynamics/ForceAccumulatorTests.cpp
    dynamics/MassPropertiesTests.cpp
    dynamics/TranslationalDynamicsModelTests.cpp
    dynamics/ConstantGravityTests.cpp
    dynamics/RigidBodyStateTests.cpp
    dynamics/InertiaPropertiesTests.cpp
    dynamics/MomentAccumulatorTests.cpp
    dynamics/RotationalKinematicsTests.cpp
    dynamics/RotationalDynamicsTests.cpp
    dynamics/RigidBodyQuaternionNormalizationTests.cpp
    dynamics/SixDoFDynamicsModelTests.cpp
    environment/AtmosphereStateTests.cpp
    environment/AtmosphereModelTests.cpp
    environment/StandardAtmosphereTests.cpp
    environment/ConstantWindModelTests.cpp
    environment/WindModelTests.cpp
    environment/RelativeAirVelocityTests.cpp
    aerodynamics/AerodynamicAnglesTests.cpp
    aerodynamics/AerodynamicCoefficientsTests.cpp
    aerodynamics/AerodynamicFrameTransformTests.cpp
    aerodynamics/AerodynamicModelTests.cpp
    aerodynamics/AerodynamicReferenceTests.cpp
    aerodynamics/AirDataTests.cpp
    aerodynamics/SimpleAerodynamicModelTests.cpp
    integration/MathGeometryIntegrationTests.cpp
    integration/NumericalAccuracyTests.cpp
    integration/SimulationCoreIntegrationTests.cpp
    integration/BallisticAnalyticalTests.cpp
    integration/TranslationalDynamicsIntegrationTests.cpp
    integration/AtmosphereValidationTests.cpp
    integration/EnvironmentIntegrationTests.cpp
    integration/AerodynamicIntegrationTests.cpp
    integration/SixDoFAnalyticalTests.cpp
    integration/SixDoFIntegrationTests.cpp
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