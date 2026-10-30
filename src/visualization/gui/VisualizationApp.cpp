#include "VisualizationApp.h"
#include "GuiTheme.h"
#include "panels/DashboardPanel.h"
#include "panels/PlaybackPanel.h"
#include "panels/TelemetryPanel.h"
#include "panels/TrajectoryPanel.h"
#include "panels/Vehicle3DPanel.h"
#include "panels/EventsPanel.h"
#include "panels/PerformancePanel.h"
#include "panels/HilDashboardPanel.h"
#include <hil/HilMonitoredRun.h>
#include <hil/HilHardwareRun.h>
#include <visualization/SixDoFLiveSimulation.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <exception>
#include <thread>

namespace aeroforge::visualization::gui
{
    namespace
    {
        constexpr char layoutFile[] = "AeroForgeVisualizer.ini";
        // Test-only samples; not produced by AeroForge's flight dynamics.
        VisualizationSnapshot demoSnapshot(double t)
        {
            using math::Vector3;
            const double east = 2.0 * std::sin(0.5 * t);
            const double north = 5.0 * t;
            const double down = -0.3 * t;
            const auto q = math::Quaternion::fromRotationZ(0.2 * std::sin(0.4 * t));
            return { t,Vector3(north,east,down),
                Vector3(5.0,std::cos(0.5 * t),-0.3),q,Vector3(0,0,0.08 * std::cos(0.4 * t)),
                Vector3(north + 0.1,east - 0.1,down),Vector3(5.0,std::cos(0.5 * t),-0.3),
                q,Vector3(0,0,0.08 * std::cos(0.4 * t)),Vector3(),Vector3(),Vector3() };
        }
    }
    VisualizationApp::~VisualizationApp() { shutdown(); }
    void VisualizationApp::glfwErrorCallback(int code, const char* text)
    {
        std::fprintf(stderr, "GLFW error %d: %s\n", code, text ? text : "unknown");
    }
    void VisualizationApp::dropCallback(GLFWwindow* window, int count, const char** paths)
    {
        auto* app = static_cast<VisualizationApp*>(glfwGetWindowUserPointer(window));
        if (app && count > 0 && paths && paths[0])app->loadRecording(paths[0]);
    }
    void VisualizationApp::loadRecording(const char* path) noexcept
    {
        stopHilLive();
        stopRealLive();
        stopSyntheticLive();
        try { session_.loadRecording(path);notification_ = "Loaded telemetry recording."; }
        catch (const std::exception& e) { notification_ = e.what(); }
    }
    void VisualizationApp::loadEvents(const char* path) noexcept
    {
        try { session_.loadEventCsv(path);notification_ = "Loaded event annotations."; }
        catch (const std::exception& e) { notification_ = e.what(); }
    }
    void VisualizationApp::stopSyntheticLive() noexcept
    {
        if (demoThread_.joinable())
        {
            demoThread_.request_stop();
            demoThread_.join();
        }
        demoPublisher_.reset();
    }
    void VisualizationApp::startSyntheticLive()
    {
        stopHilLive();
        stopRealLive();
        stopSyntheticLive();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        demoPublisher_ = publisher;
        demoThread_ = std::jthread([publisher](std::stop_token stop)
            {
                const auto start = std::chrono::steady_clock::now();
                while (!stop.stop_requested())
                {
                    const auto now = std::chrono::steady_clock::now();
                    const double t = std::chrono::duration<double>(now - start).count();
                    publisher->publish(demoSnapshot(t));
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            });
        notification_ = "SYNTHETIC live publisher active (not a flight simulation).";
    }


    void VisualizationApp::stopRealLive() noexcept
    {
        if (realThread_.joinable())
        {
            realThread_.request_stop();
            realThread_.join();
        }

        realPublisher_.reset();
    }

    void VisualizationApp::startRealLive()
    {
        stopHilLive();
        // Stop either previous live source before replacing it.
        stopSyntheticLive();
        stopRealLive();

        auto publisher = std::make_shared<LiveTelemetryPublisher>();

        // The GUI reads the latest snapshot through VisualizationSession.
        session_.attachLive(publisher);
        realPublisher_ = publisher;

        // Simulation runs independently from the OpenGL/ImGui thread.
        realThread_ = std::jthread(
            [publisher](std::stop_token stop)
            {
                try
                {
                    runSixDoFLiveSimulation(
                        *publisher,
                        stop,
                        0.01,  // 100 Hz simulation timestep
                        1000,  // 10 simulated seconds
                        true   // Real-time pacing
                    );
                }
                catch (const std::exception& e)
                {
                    std::fprintf(
                        stderr,
                        "6DOF live simulation failed: %s\n",
                        e.what()
                    );
                }
                catch (...)
                {
                    std::fprintf(
                        stderr,
                        "6DOF live simulation failed: unknown error\n"
                    );
                }
            }
        );

        notification_ =
            "Live 6DOF simulation started. "
            "Using actual RK4-integrated physics.";
    }

    void VisualizationApp::stopHilLive() noexcept
    {
        if (hilThread_.joinable())
        {
            hilThread_.request_stop();
            hilThread_.join();
        }
        hilPublisher_.reset();
        // Retain previous measurements for inspection.
    }

    void VisualizationApp::startHilLive()
    {
        stopSyntheticLive();
        stopRealLive();
        stopHilLive();

        auto store = std::make_shared<hil::HilTelemetryStore>();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        hilTelemetry_ = store;
        hilPublisher_ = publisher;
        panels_.hilMonitor = true;

        hilThread_ = std::jthread([store, publisher](std::stop_token token)
            {
                try
                {
                    hil::HilMonitoredRunConfig config{};
                    config.stepS = 0.01;
                    config.steps = 12000;
                    config.realtimePacing = true;
                    hil::runHilMonitoredLoopback(
                        *store, publisher.get(), token, config);
                }
                catch (const std::exception& e)
                {
                    std::fprintf(stderr, "Software HIL failed: %s\n", e.what());
                    hil::HilHealthSnapshot failed{};
                    failed.mode = hil::HilLinkMode::SOFTWARE_LOOPBACK;
                    failed.watchdogState = hil::HilWatchdogState::FAULTED;
                    store->publishHealth(failed);
                }
            });
        notification_ = "Software HIL loopback started (no external controller).";
    }

    void VisualizationApp::startHilHardwareLive()
    {
#ifndef _WIN32
        notification_ = "Physical COM-port HIL requires Windows.";
#else
        stopSyntheticLive();
        stopRealLive();
        stopHilLive();

        auto store = std::make_shared<hil::HilTelemetryStore>();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        hilTelemetry_ = store;
        hilPublisher_ = publisher;
        panels_.hilMonitor = true;

        hilThread_ = std::jthread([store, publisher](std::stop_token token)
            {
                try
                {
                    hil::HilHardwareRunConfig config{};
                    config.portName = L"COM3";
                    config.baudRate = 115200;
                    config.stepS = 0.01;
                    config.steps = 12000;
                    hil::runHilHardwareClosedLoop(
                        *store, publisher.get(), token, config);
                }
                catch (const std::exception& e)
                {
                    std::fprintf(stderr, "Physical HIL failed: %s\n", e.what());
                    hil::HilHealthSnapshot failed{};
                    failed.mode = hil::HilLinkMode::SERIAL;
                    failed.watchdogState = hil::HilWatchdogState::FAULTED;
                    failed.serialState = hil::HilSerialWorkerState::FAILED;
                    store->publishHealth(failed);
                }
            });
        notification_ = "Hardware HIL requested on COM3 at 115200 baud.";
#endif
    }

    bool VisualizationApp::initialize()
    {
        glfwSetErrorCallback(&VisualizationApp::glfwErrorCallback);
        if (glfwInit() != GLFW_TRUE)return false;
        glfwInitialized_ = true;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
        window_ = glfwCreateWindow(1440, 900, "AeroForge Visualizer", nullptr, nullptr);
        if (!window_)return false;
        glfwMakeContextCurrent(window_);
        glfwSetWindowUserPointer(window_, this);
        glfwSetDropCallback(window_, &VisualizationApp::dropCallback);
        glfwSwapInterval(1);
        IMGUI_CHECKVERSION();ImGui::CreateContext();imguiContextCreated_ = true;
        ImPlot::CreateContext();implotContextCreated_ = true;
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.IniFilename = layoutFile;
        applyDefaultLayout_ = !std::filesystem::exists(layoutFile);
        applyAeroForgeTheme();
        if (!ImGui_ImplGlfw_InitForOpenGL(window_, true))return false;
        glfwBackendInitialized_ = true;
        if (!ImGui_ImplOpenGL3_Init("#version 330"))return false;
        openglBackendInitialized_ = true;
        return true;
    }
    int VisualizationApp::run()
    {
        if (!window_ || !openglBackendInitialized_)return EXIT_FAILURE;
        using clock = std::chrono::steady_clock;
        auto previous = clock::now();
        while (glfwWindowShouldClose(window_) == GLFW_FALSE)
        {
            glfwPollEvents();
            const auto now = clock::now();
            const double delta = std::chrono::duration<double>(now - previous).count();
            previous = now;
            if (std::isfinite(delta) && delta > 0)
            {
                frameStats_.addFrameSeconds(delta);
                session_.update(delta);
            }
            int w = 0, h = 0;glfwGetFramebufferSize(window_, &w, &h);
            if (w <= 0 || h <= 0) { glfwWaitEventsTimeout(0.1);continue; }
            renderFrame();
            ImGui::Render();
            glViewport(0, 0, w, h);
            glClearColor(0.065f, 0.080f, 0.104f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window_);
        }
        return EXIT_SUCCESS;
    }
    void VisualizationApp::drawMenuBar()
    {
        if (!ImGui::BeginMainMenuBar())return;
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open telemetry CSV..."))showLoadDialog_ = true;
            if (ImGui::MenuItem("Open events CSV..."))showEventDialog_ = true;
            if (ImGui::MenuItem("Start synthetic live demo"))startSyntheticLive();
            if (ImGui::MenuItem("Start 6DOF live simulation"))
            {
                startRealLive();
            }
            if (ImGui::MenuItem("Start HIL software loopback"))
            {
                startHilLive();
            }
#ifdef _WIN32
            if (ImGui::MenuItem("Start HIL hardware (COM3)"))
            {
                startHilHardwareLive();
            }
#endif
            if (ImGui::MenuItem("Stop live source"))
            {
                stopHilLive();
                stopSyntheticLive();
                stopRealLive();
                session_.stopLive();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit"))glfwSetWindowShouldClose(window_, GLFW_TRUE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Dashboard", nullptr, &panels_.dashboard);
            ImGui::MenuItem("Playback", nullptr, &panels_.playback);
            ImGui::MenuItem("Telemetry", nullptr, &panels_.telemetry);
            ImGui::MenuItem("Trajectory", nullptr, &panels_.trajectory);
            ImGui::MenuItem("Vehicle 3D", nullptr, &panels_.vehicle3D);
            ImGui::MenuItem("Events", nullptr, &panels_.events);
            ImGui::MenuItem("Performance", nullptr, &panels_.performance);
            ImGui::MenuItem("HIL Monitor", nullptr, &panels_.hilMonitor);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &showImGuiDemo_);
            ImGui::MenuItem("ImPlot Demo", nullptr, &showImPlotDemo_);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Layout"))
        {
            if (ImGui::MenuItem("Show all panels"))panels_ = PanelVisibility{};
            if (ImGui::MenuItem("Reset default layout"))
            {
                panels_ = PanelVisibility{};applyDefaultLayout_ = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
    void VisualizationApp::resetDefaultLayout(unsigned int id)
    {
        const auto* view = ImGui::GetMainViewport();
        ImGui::DockBuilderRemoveNode(id);
        ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(id, view->WorkSize);
        ImGuiID center = id;
        ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.23f, nullptr, &center);
        ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f, nullptr, &center);
        ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.34f, nullptr, &center);
        ImGuiID lb = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.40f, nullptr, &left);
        ImGuiID rb = ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.38f, nullptr, &right);
        ImGui::DockBuilderDockWindow("Dashboard", left);
        ImGui::DockBuilderDockWindow("Playback", lb);
        ImGui::DockBuilderDockWindow("Trajectory", center);
        ImGui::DockBuilderDockWindow("Vehicle 3D", center);
        ImGui::DockBuilderDockWindow("Telemetry", bottom);
        ImGui::DockBuilderDockWindow("Events", right);
        ImGui::DockBuilderDockWindow("Performance", rb);
        ImGui::DockBuilderDockWindow("HIL Monitor", right);
        ImGui::DockBuilderFinish(id);
        applyDefaultLayout_ = false;
    }
    void VisualizationApp::drawDockspace()
    {
        const auto* view = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(view->WorkPos);
        ImGui::SetNextWindowSize(view->WorkSize);
        ImGui::SetNextWindowViewport(view->ID);
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking |
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("AeroForge Dock Host", nullptr, flags);
        ImGui::PopStyleVar(3);
        const ImGuiID id = ImGui::GetID("AeroForgeDockspace");
        if (applyDefaultLayout_)resetDefaultLayout(id);
        ImGui::DockSpace(id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::End();
    }
    void VisualizationApp::renderFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawMenuBar();drawDockspace();
        if (panels_.dashboard)panels::drawDashboardPanel(&panels_.dashboard, session_);
        if (panels_.playback)panels::drawPlaybackPanel(&panels_.playback, session_);
        if (panels_.telemetry)panels::drawTelemetryPanel(&panels_.telemetry, session_);
        if (panels_.trajectory)panels::drawTrajectoryPanel(&panels_.trajectory, session_);
        if (panels_.vehicle3D)panels::drawVehicle3DPanel(&panels_.vehicle3D, session_, camera_);
        if (panels_.events)panels::drawEventsPanel(&panels_.events, session_);
        if (panels_.performance)panels::drawPerformancePanel(&panels_.performance, session_, frameStats_);
        if (panels_.hilMonitor)panels::drawHilDashboardPanel(&panels_.hilMonitor, hilTelemetry_.get());
        if (showImGuiDemo_)ImGui::ShowDemoWindow(&showImGuiDemo_);
        if (showImPlotDemo_)ImPlot::ShowDemoWindow(&showImPlotDemo_);
        if (showLoadDialog_)
        {
            if (ImGui::Begin("Open telemetry CSV", &showLoadDialog_))
            {
                ImGui::InputText("Path", filePath_, sizeof(filePath_));
                if (ImGui::Button("Load recording"))
                {
                    loadRecording(filePath_);showLoadDialog_ = false;
                }
                ImGui::TextWrapped("Alternatively, drag a CSV file onto the window.");
            }
            ImGui::End();
        }
        if (showEventDialog_)
        {
            if (ImGui::Begin("Open event annotations", &showEventDialog_))
            {
                ImGui::InputText("Path", eventPath_, sizeof(eventPath_));
                if (ImGui::Button("Load events"))
                {
                    loadEvents(eventPath_);showEventDialog_ = false;
                }
            }
            ImGui::End();
        }
        if (!notification_.empty())
        {
            ImGui::SetNextWindowBgAlpha(0.95f);
            if (ImGui::Begin("Status##AeroForge", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::TextWrapped("%s", notification_.c_str());
                if (ImGui::Button("Dismiss"))notification_.clear();
            }
            ImGui::End();
        }
    }
    void VisualizationApp::shutdown() noexcept
    {
        stopHilLive();
        stopRealLive();
        stopSyntheticLive();
        if (window_)glfwMakeContextCurrent(window_);
        if (openglBackendInitialized_) { ImGui_ImplOpenGL3_Shutdown();openglBackendInitialized_ = false; }
        if (glfwBackendInitialized_) { ImGui_ImplGlfw_Shutdown();glfwBackendInitialized_ = false; }
        if (implotContextCreated_) { ImPlot::DestroyContext();implotContextCreated_ = false; }
        if (imguiContextCreated_) { ImGui::DestroyContext();imguiContextCreated_ = false; }
        if (window_) { glfwDestroyWindow(window_);window_ = nullptr; }
        if (glfwInitialized_) { glfwTerminate();glfwInitialized_ = false; }
    }
}
