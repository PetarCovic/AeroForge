
cmake_minimum_required(VERSION 3.20)

project(
    AeroForge
    VERSION 0.1.0
    DESCRIPTION "6DOF aerospace simulation, guidance, navigation, and control platform"
    LANGUAGES CXX
)

option(
    AEROFORGE_BUILD_VISUALIZATION
    "Build the AeroForge visualization application"
    OFF
)

#
# Core simulation library
#
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
    src/simulation/RealtimeExecutionConfig.cpp
    src/simulation/SimulationPacer.cpp
    src/simulation/RealtimeSimulationRunner.cpp

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

    src/actuators/ThrottleActuator.cpp
    src/actuators/GimbalActuator.cpp
    src/actuators/BodyMomentActuator.cpp
    src/actuators/GimbaledThrust.cpp

    src/gnc/AttitudeSetpoint.cpp
    src/gnc/HoldAttitudeGuidance.cpp
    src/gnc/AttitudeError.cpp
    src/gnc/AttitudePdController.cpp

    src/failures/FaultSchedule.cpp
    src/failures/SensorFaultInjector.cpp
    src/failures/GpsFaultInjector.cpp
    src/failures/ActuatorFaultInjector.cpp

    src/montecarlo/ParameterDistribution.cpp
    src/montecarlo/MonteCarloConfig.cpp
    src/montecarlo/MonteCarloStatistics.cpp
    src/montecarlo/MonteCarloRunner.cpp

    src/reproducibility/SeedManager.cpp
    src/reproducibility/ReplayConfig.cpp

    src/telemetry/TelemetryRecorder.cpp
    src/telemetry/TelemetrySchedule.cpp
    src/telemetry/CsvTelemetryWriter.cpp
    src/telemetry/RunMetadata.cpp
    src/telemetry/EventLogger.cpp
    src/telemetry/FlightAnalyzer.cpp
    src/telemetry/GncAnalyzer.cpp
    src/telemetry/MonteCarloReport.cpp

    src/performance/PerformanceTimer.cpp
    src/performance/RealtimeFactor.cpp
    src/performance/PerformanceSummary.cpp

    src/visualization/PlaybackController.cpp
    src/visualization/VisualizationInterpolator.cpp
    src/visualization/RecordedTelemetryDataSource.cpp
    src/visualization/LiveTelemetryPublisher.cpp
    src/visualization/SixDoFLiveSimulation.cpp
    src/visualization/VisualizationModel.cpp
    src/visualization/TrajectoryHistory.cpp
    src/visualization/LiveVisualizationModel.cpp
    src/visualization/Camera3D.cpp
    src/visualization/FrameTimingStats.cpp
    src/visualization/PlotDataBuilder.cpp
    src/visualization/TelemetryCsvLoader.cpp
    src/visualization/VisualizationSession.cpp
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

#
# Command-line executable
#
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

#
# Optional graphical visualization application
#
if(AEROFORGE_BUILD_VISUALIZATION)

    include(FetchContent)

    find_package(OpenGL REQUIRED)

    #
    # GLFW
    #
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.5.1
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(glfw)

    #
    # Dear ImGui (Docking)
    #
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.9b-docking
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(imgui)

    add_library(AeroForgeImGui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp

        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )

    target_include_directories(AeroForgeImGui
        PUBLIC
            ${imgui_SOURCE_DIR}
            ${imgui_SOURCE_DIR}/backends
    )

    target_link_libraries(AeroForgeImGui
        PUBLIC
            glfw
            OpenGL::GL
    )

    #
    # ImPlot
    #
    FetchContent_Declare(
        implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG v1.0
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(implot)

    add_library(AeroForgeImPlot STATIC
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
        ${implot_SOURCE_DIR}/implot_demo.cpp
    )

    target_include_directories(AeroForgeImPlot
        PUBLIC
            ${implot_SOURCE_DIR}
    )

    target_link_libraries(AeroForgeImPlot
        PUBLIC
            AeroForgeImGui
    )

    #
    # AeroForge visualization application
    #
    add_executable(AeroForgeVisualizer
        #
        # Application
        #
        src/visualization/gui/main.cpp
        src/visualization/gui/VisualizationApp.h
        src/visualization/gui/VisualizationApp.cpp
        src/visualization/gui/PanelVisibility.h
        src/visualization/gui/GuiTheme.h
        src/visualization/gui/GuiTheme.cpp

        #
        # Interface state and appearance
        #
        src/visualization/gui/PanelVisibility.h
        src/visualization/gui/GuiTheme.h
        src/visualization/gui/GuiTheme.cpp

        #
        # Dashboard panels
        #
        src/visualization/gui/panels/DashboardPanel.h
        src/visualization/gui/panels/DashboardPanel.cpp

        src/visualization/gui/panels/PlaybackPanel.h
        src/visualization/gui/panels/PlaybackPanel.cpp

        src/visualization/gui/panels/TelemetryPanel.h
        src/visualization/gui/panels/TelemetryPanel.cpp

        src/visualization/gui/panels/TrajectoryPanel.h
        src/visualization/gui/panels/TrajectoryPanel.cpp

        src/visualization/gui/panels/Vehicle3DPanel.h
        src/visualization/gui/panels/Vehicle3DPanel.cpp

        src/visualization/gui/panels/EventsPanel.h
        src/visualization/gui/panels/EventsPanel.cpp

        src/visualization/gui/panels/PerformancePanel.h
        src/visualization/gui/panels/PerformancePanel.cpp

        #
        # Rendering
        #
        src/visualization/gui/rendering/VehicleRenderer.h
        src/visualization/gui/rendering/VehicleRenderer.cpp
    )

    target_link_libraries(AeroForgeVisualizer
        PRIVATE
            AeroForgeCore
            AeroForgeImGui
            AeroForgeImPlot
            glfw
            OpenGL::GL
    )

    target_compile_features(AeroForgeVisualizer
        PRIVATE
            cxx_std_20
    )

    if(MSVC)
        target_compile_options(AeroForgeVisualizer
            PRIVATE
                /W4
                /permissive-
                /Zc:__cplusplus
        )
    else()
        target_compile_options(AeroForgeVisualizer
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
        )
    endif()

endif()

#
# Automated tests
#
include(CTest)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

#
# Performance benchmarks and simulation examples
#
add_subdirectory(benchmarks)
add_subdirectory(examples)

#
# Debug symbols for Release profiling on MSVC
#
if(MSVC)

    target_compile_options(AeroForgeCore
        PRIVATE
            $<$<CONFIG:Release>:/Zi>
    )

    target_compile_options(AeroForgeBenchmarks
        PRIVATE
            $<$<CONFIG:Release>:/Zi>
    )

    target_link_options(AeroForgeBenchmarks
        PRIVATE
            $<$<CONFIG:Release>:/DEBUG>
    )

endif()