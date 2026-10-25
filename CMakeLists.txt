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
    src/simulation/State.cpp
    src/simulation/EulerIntegrator.cpp
    src/simulation/RK4Integrator.cpp
    src/simulation/SimulationClock.cpp
    src/simulation/SimulationRunner.cpp
    src/dynamics/TranslationalState.cpp
    src/dynamics/ForceAccumulator.cpp
    src/dynamics/MassProperties.cpp
    src/dynamics/TranslationalDynamicsModel.cpp
    src/dynamics/ConstantGravity.cpp
    src/dynamics/RigidBodyState.cpp
    src/dynamics/InertiaProperties.cpp
    src/dynamics/MomentAccumulator.cpp
    src/dynamics/RotationalKinematics.cpp
    src/dynamics/RotationalDynamics.cpp
    src/dynamics/RigidBodyQuaternionNormalization.cpp
    src/dynamics/SixDoFDynamicsModel.cpp
    src/environment/AtmosphereState.cpp
    src/environment/StandardAtmosphere.cpp
    src/environment/ConstantWindModel.cpp
    src/environment/RelativeAirVelocity.cpp
    src/aerodynamics/AerodynamicAngles.cpp
    src/aerodynamics/AerodynamicCoefficients.cpp
    src/aerodynamics/AerodynamicFrameTransform.cpp
    src/aerodynamics/AerodynamicReference.cpp
    src/aerodynamics/AirData.cpp
    src/aerodynamics/SimpleAerodynamicModel.cpp
    src/rocket/PropellantTank.cpp
    src/rocket/RocketMassProperties.cpp
    src/rocket/ThrustCurve.cpp
    src/rocket/RocketEngine.cpp
    src/rocket/ThrustMoment.cpp
    src/rocket/PropellantDepletion.cpp
    src/rocket/RocketDynamicsModel.cpp
    src/sensors/AccelerometerModel.cpp
    src/sensors/GyroscopeModel.cpp
    src/sensors/ImuModel.cpp
    src/sensors/GpsModel.cpp
    src/sensors/SensorSchedule.cpp
    src/navigation/NavigationState.cpp
    src/navigation/InertialNavigator.cpp
    src/navigation/GpsCorrector.cpp
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