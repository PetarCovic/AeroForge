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

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

namespace aeroforge::visualization::gui
{
    namespace
    {
        constexpr char layoutFile[] = "AeroForgeVisualizer.ini";
        constexpr float statusBarHeight = 34.0f;
        const ImVec4 goodColor{ 0.36f, 0.85f, 0.68f, 1.0f };
        const ImVec4 warnColor{ 1.00f, 0.74f, 0.36f, 1.0f };
        const ImVec4 badColor{ 1.00f, 0.45f, 0.45f, 1.0f };

        // This is UI-only test data, deliberately NOT flight dynamics.
        VisualizationSnapshot demoSnapshot(double t)
        {
            using math::Vector3;
            const double east = 2.0 * std::sin(0.5 * t);
            const double north = 5.0 * t;
            const double down = -0.3 * t;
            const auto q = math::Quaternion::fromRotationZ(0.2 * std::sin(0.4 * t));
            return { t, Vector3(north, east, down),
                Vector3(5.0, std::cos(0.5 * t), -0.3), q,
                Vector3(0, 0, 0.08 * std::cos(0.4 * t)),
                Vector3(north + 0.1, east - 0.1, down),
                Vector3(5.0, std::cos(0.5 * t), -0.3), q,
                Vector3(0, 0, 0.08 * std::cos(0.4 * t)),
                Vector3(), Vector3(), Vector3() };
        }

        bool isComPort(const std::string& port)
        {
            if (port.size() < 4 || port.rfind("COM", 0) != 0) return false;
            return std::all_of(port.begin() + 3, port.end(), [](unsigned char c)
                { return std::isdigit(c) != 0; }) &&
                port.find_first_not_of('0', 3) != std::string::npos;
        }
    }

    VisualizationApp::~VisualizationApp() { shutdown(); }

    void VisualizationApp::glfwErrorCallback(int code, const char* message)
    {
        std::fprintf(stderr, "GLFW error %d: %s\n", code,
            message ? message : "unknown");
    }

    void VisualizationApp::dropCallback(GLFWwindow* window, int count, const char** paths)
    {
        auto* app = static_cast<VisualizationApp*>(glfwGetWindowUserPointer(window));
        if (app && count > 0 && paths && paths[0]) app->loadRecording(paths[0]);
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

    void VisualizationApp::stopRealLive() noexcept
    {
        if (realThread_.joinable())
        {
            realThread_.request_stop();
            realThread_.join();
        }
        realPublisher_.reset();
    }

    void VisualizationApp::stopHilLive() noexcept
    {
        if (hilThread_.joinable())
        {
            hilThread_.request_stop();
            hilThread_.join();
        }
        hilPublisher_.reset();
        // Preserve hilTelemetry_ so stopped-run diagnostics remain inspectable.
    }

    void VisualizationApp::stopLiveSource() noexcept
    {
        stopHilLive();
        stopRealLive();
        stopSyntheticLive();
        session_.stopLive();
        if (source_ != SourceKind::Recording) source_ = SourceKind::None;
        workerStatus_.store(WorkerStatus::Idle);
        std::lock_guard<std::mutex> lock(workerErrorMutex_);
        workerError_.clear();
    }

    void VisualizationApp::failWorker(const std::string& description)
    {
        {
            std::lock_guard<std::mutex> lock(workerErrorMutex_);
            workerError_ = description;
        }
        workerStatus_.store(WorkerStatus::Failed);
        std::fprintf(stderr, "AeroForge worker failed: %s\n", description.c_str());
    }

    std::string VisualizationApp::workerError() const
    {
        std::lock_guard<std::mutex> lock(workerErrorMutex_);
        return workerError_;
    }

    void VisualizationApp::loadRecording(const char* path) noexcept
    {
        stopLiveSource();
        try
        {
            session_.loadRecording(path);
            source_ = SourceKind::Recording;
            notification_ = "Telemetry recording loaded.";
            notificationIsError_ = false;
        }
        catch (const std::exception& e)
        {
            source_ = session_.mode() == VisualizationSession::Mode::Recorded
                ? SourceKind::Recording : SourceKind::None;
            notification_ = std::string("Cannot load recording: ") + e.what();
            notificationIsError_ = true;
        }
    }

    void VisualizationApp::loadEvents(const char* path) noexcept
    {
        try
        {
            session_.loadEventCsv(path);
            notification_ = "Event annotations loaded.";
            notificationIsError_ = false;
        }
        catch (const std::exception& e)
        {
            notification_ = std::string("Cannot load events: ") + e.what();
            notificationIsError_ = true;
        }
    }

    void VisualizationApp::startSyntheticLive()
    {
        stopLiveSource();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        demoPublisher_ = publisher;
        source_ = SourceKind::Synthetic;
        workerStatus_.store(WorkerStatus::Running);
        demoThread_ = std::jthread([this, publisher](std::stop_token stop)
            {
                try
                {
                    const auto start = std::chrono::steady_clock::now();
                    while (!stop.stop_requested())
                    {
                        const auto now = std::chrono::steady_clock::now();
                        const double t = std::chrono::duration<double>(now - start).count();
                        publisher->publish(demoSnapshot(t));
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                }
                catch (const std::exception& e) { failWorker(e.what()); }
                catch (...) { failWorker("Synthetic telemetry source failed: unknown error"); }
            });
        notification_ = "Synthetic display test active (NOT flight simulation).";
        notificationIsError_ = false;
    }

    void VisualizationApp::startRealLive()
    {
        stopLiveSource();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        realPublisher_ = publisher;
        source_ = SourceKind::SixDoF;
        workerStatus_.store(WorkerStatus::Running);
        realThread_ = std::jthread([this, publisher](std::stop_token stop)
            {
                try
                {
                    runSixDoFLiveSimulation(*publisher, stop, 0.01, 1000, true);
                    if (!stop.stop_requested()) workerStatus_.store(WorkerStatus::Completed);
                }
                catch (const std::exception& e) { failWorker(e.what()); }
                catch (...) { failWorker("6DOF simulation failed: unknown error"); }
            });
        notification_ = "6DOF RK4 simulation started (10 simulated seconds).";
        notificationIsError_ = false;
    }

    void VisualizationApp::startHilLive()
    {
        stopLiveSource();
        auto store = std::make_shared<hil::HilTelemetryStore>();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        hilTelemetry_ = store;
        hilPublisher_ = publisher;
        panels_.hilMonitor = true;
        source_ = SourceKind::SoftwareHIL;
        workerStatus_.store(WorkerStatus::Running);
        hilThread_ = std::jthread([this, store, publisher](std::stop_token stop)
            {
                try
                {
                    hil::HilMonitoredRunConfig config{};
                    config.stepS = 0.01;
                    config.steps = 12000;
                    config.realtimePacing = true;
                    hil::runHilMonitoredLoopback(*store, publisher.get(), stop, config);
                    if (!stop.stop_requested()) workerStatus_.store(WorkerStatus::Completed);
                }
                catch (const std::exception& e)
                {
                    hil::HilHealthSnapshot failed{};
                    failed.mode = hil::HilLinkMode::SOFTWARE_LOOPBACK;
                    failed.watchdogState = hil::HilWatchdogState::FAULTED;
                    store->publishHealth(failed);
                    failWorker(e.what());
                }
                catch (...) { failWorker("Software HIL failed: unknown error"); }
            });
        notification_ = "Software HIL loopback started (no external controller).";
        notificationIsError_ = false;
    }

    void VisualizationApp::startHilHardwareLive()
    {
#ifndef _WIN32
        notification_ = "Physical COM-port HIL requires Windows.";
        notificationIsError_ = true;
#else
        std::string port = hilPort_;
        std::transform(port.begin(), port.end(), port.begin(), [](unsigned char c)
            { return static_cast<char>(std::toupper(c)); });
        if (!isComPort(port) || hilBaudRate_ <= 0 || hilBaudRate_ > 4000000)
        {
            notification_ = "Enter a COM port such as COM3 and a valid positive baud rate.";
            notificationIsError_ = true;
            return;
        }
        const auto baud = static_cast<std::uint32_t>(hilBaudRate_);
        const std::wstring widePort(port.begin(), port.end());
        stopLiveSource();
        auto store = std::make_shared<hil::HilTelemetryStore>();
        auto publisher = std::make_shared<LiveTelemetryPublisher>();
        session_.attachLive(publisher);
        hilTelemetry_ = store;
        hilPublisher_ = publisher;
        panels_.hilMonitor = true;
        source_ = SourceKind::HardwareHIL;
        workerStatus_.store(WorkerStatus::Running);
        hilThread_ = std::jthread([this, store, publisher, widePort, baud](std::stop_token stop)
            {
                try
                {
                    hil::HilHardwareRunConfig config{};
                    config.portName = widePort;
                    config.baudRate = baud;
                    config.stepS = 0.01;
                    config.steps = 12000;
                    const auto result = hil::runHilHardwareClosedLoop(*store, publisher.get(), stop, config);
                    if (!stop.stop_requested())
                    {
                        if (result.serialFailed) failWorker("Physical HIL transport failed.");
                        else workerStatus_.store(WorkerStatus::Completed);
                    }
                }
                catch (const std::exception& e)
                {
                    hil::HilHealthSnapshot failed{};
                    failed.mode = hil::HilLinkMode::SERIAL;
                    failed.watchdogState = hil::HilWatchdogState::FAULTED;
                    failed.serialState = hil::HilSerialWorkerState::FAILED;
                    store->publishHealth(failed);
                    failWorker(e.what());
                }
                catch (...) { failWorker("Physical HIL failed: unknown error"); }
            });
        notification_ = "Experimental physical HIL requested on " + port + " at " +
            std::to_string(baud) + " baud.";
        notificationIsError_ = false;
#endif
    }

    const char* VisualizationApp::sourceLabel() const noexcept
    {
        switch (source_)
        {
        case SourceKind::Recording: return "Recording";
        case SourceKind::Synthetic: return "Synthetic test";
        case SourceKind::SixDoF: return "6DOF physics";
        case SourceKind::SoftwareHIL: return "Software HIL";
        case SourceKind::HardwareHIL: return "Hardware HIL (experimental)";
        default: return "No source";
        }
    }

    const char* VisualizationApp::statusLabel() const noexcept
    {
        if (source_ == SourceKind::Recording)
            return session_.isPlaying() ? "Playing" : "Paused";
        switch (workerStatus_.load())
        {
        case WorkerStatus::Running: return "Running";
        case WorkerStatus::Completed: return "Completed";
        case WorkerStatus::Failed: return "FAILED";
        default: return "Idle";
        }
    }

    bool VisualizationApp::initialize()
    {
        glfwSetErrorCallback(&VisualizationApp::glfwErrorCallback);
        if (glfwInit() != GLFW_TRUE) return false;
        glfwInitialized_ = true;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
        window_ = glfwCreateWindow(1440, 900, "AeroForge | Engineering Visualizer", nullptr, nullptr);
        if (!window_) return false;
        glfwMakeContextCurrent(window_);
        glfwSetWindowUserPointer(window_, this);
        glfwSetDropCallback(window_, &VisualizationApp::dropCallback);
        glfwSwapInterval(1);
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        imguiContextCreated_ = true;
        ImPlot::CreateContext();
        implotContextCreated_ = true;
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        io.IniFilename = layoutFile;
        applyDefaultLayout_ = !std::filesystem::exists(layoutFile);
        applyAeroForgeTheme();
        if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) return false;
        glfwBackendInitialized_ = true;
        if (!ImGui_ImplOpenGL3_Init("#version 330")) return false;
        openglBackendInitialized_ = true;
        return true;
    }

    int VisualizationApp::run()
    {
        if (!window_ || !openglBackendInitialized_) return EXIT_FAILURE;
        using clock = std::chrono::steady_clock;
        auto previous = clock::now();
        while (glfwWindowShouldClose(window_) == GLFW_FALSE)
        {
            glfwPollEvents();
            const auto now = clock::now();
            const double delta = std::chrono::duration<double>(now - previous).count();
            previous = now;
            if (std::isfinite(delta) && delta > 0.0)
            {
                frameStats_.addFrameSeconds(delta);
                session_.update(delta);
            }
            int w = 0, h = 0;
            glfwGetFramebufferSize(window_, &w, &h);
            if (w <= 0 || h <= 0) { glfwWaitEventsTimeout(0.1); continue; }
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
        if (!ImGui::BeginMainMenuBar()) return;
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open telemetry CSV...")) showLoadDialog_ = true;
            if (ImGui::MenuItem("Open event CSV...")) showEventDialog_ = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) glfwSetWindowShouldClose(window_, GLFW_TRUE);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simulation"))
        {
            if (ImGui::MenuItem("Start 6DOF simulation")) startRealLive();
            if (ImGui::MenuItem("Start synthetic display test")) startSyntheticLive();
            ImGui::Separator();
            const bool live = session_.mode() == VisualizationSession::Mode::Live;
            if (ImGui::MenuItem("Stop live source", nullptr, false, live))
            {
                stopLiveSource();
                notification_ = "Live source stopped.";
                notificationIsError_ = false;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("HIL"))
        {
            if (ImGui::MenuItem("Start software loopback")) startHilLive();
#ifdef _WIN32
            if (ImGui::MenuItem("Configure physical HIL...")) showHardwareDialog_ = true;
#endif
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
            if (ImGui::MenuItem("Show all panels")) panels_ = PanelVisibility{};
            if (ImGui::MenuItem("Reset default layout"))
            {
                panels_ = PanelVisibility{};
                applyDefaultLayout_ = true;
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
        ImGui::DockBuilderSetNodeSize(id,
            ImVec2(view->WorkSize.x, std::max(1.0f, view->WorkSize.y - statusBarHeight)));
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
        ImGui::SetNextWindowSize(ImVec2(view->WorkSize.x,
            std::max(1.0f, view->WorkSize.y - statusBarHeight)));
        ImGui::SetNextWindowViewport(view->ID);
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking |
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoSavedSettings;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("AeroForge Dock Host", nullptr, flags);
        ImGui::PopStyleVar(3);
        const ImGuiID id = ImGui::GetID("AeroForgeDockspace");
        if (applyDefaultLayout_) resetDefaultLayout(id);
        ImGui::DockSpace(id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::End();
    }

    void VisualizationApp::drawStatusBar()
    {
        const auto* view = ImGui::GetMainViewport();
        const float top = view->WorkPos.y + std::max(0.0f, view->WorkSize.y - statusBarHeight);
        ImGui::SetNextWindowPos(ImVec2(view->WorkPos.x, top));
        ImGui::SetNextWindowSize(ImVec2(view->WorkSize.x, statusBarHeight));
        ImGui::SetNextWindowViewport(view->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 7.0f));
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking;
        if (ImGui::Begin("AeroForge Status##fixed", nullptr, flags))
        {
            ImGui::TextUnformatted("AEROFORGE");
            ImGui::SameLine();
            ImGui::TextDisabled(" | %s | ", sourceLabel());
            ImGui::SameLine(0.0f, 0.0f);
            const bool failed = workerStatus_.load() == WorkerStatus::Failed;
            const bool good = workerStatus_.load() == WorkerStatus::Running ||
                workerStatus_.load() == WorkerStatus::Completed ||
                (source_ == SourceKind::Recording && session_.isPlaying());
            ImGui::TextColored(failed ? badColor : (good ? goodColor : warnColor),
                "%s", statusLabel());
            ImGui::SameLine();
            ImGui::TextDisabled(" | t=%.2f s | %zu samples",
                session_.currentTimeS(), session_.samples().size());
            const std::string error = workerError();
            const std::string message = !error.empty() ? error : notification_;
            if (!message.empty())
            {
                ImGui::SameLine();
                ImGui::TextColored(!error.empty() || notificationIsError_ ? badColor : goodColor,
                    " | %s", message.c_str());
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", message.c_str());
            }
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
    }

    void VisualizationApp::renderFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawMenuBar();
        drawDockspace();
        if (panels_.dashboard) panels::drawDashboardPanel(&panels_.dashboard, session_);
        if (panels_.playback) panels::drawPlaybackPanel(&panels_.playback, session_);
        if (panels_.telemetry) panels::drawTelemetryPanel(&panels_.telemetry, session_);
        if (panels_.trajectory) panels::drawTrajectoryPanel(&panels_.trajectory, session_);
        if (panels_.vehicle3D) panels::drawVehicle3DPanel(&panels_.vehicle3D, session_, camera_);
        if (panels_.events) panels::drawEventsPanel(&panels_.events, session_);
        if (panels_.performance) panels::drawPerformancePanel(&panels_.performance, session_, frameStats_);
        if (panels_.hilMonitor) panels::drawHilDashboardPanel(&panels_.hilMonitor, hilTelemetry_.get());
        if (showImGuiDemo_) ImGui::ShowDemoWindow(&showImGuiDemo_);
        if (showImPlotDemo_) ImPlot::ShowDemoWindow(&showImPlotDemo_);

        if (showLoadDialog_)
        {
            const bool visible = ImGui::Begin("Open telemetry CSV", &showLoadDialog_);
            if (visible)
            {
                ImGui::TextWrapped("Enter a telemetry CSV path or drag a telemetry CSV onto the window.");
                ImGui::SetNextItemWidth(-1);
                ImGui::InputText("##telemetryPath", filePath_, sizeof(filePath_));
                if (ImGui::Button("Load recording"))
                {
                    loadRecording(filePath_);
                    showLoadDialog_ = false;
                }
            }
            ImGui::End();
        }
        if (showEventDialog_)
        {
            const bool visible = ImGui::Begin("Open event annotations", &showEventDialog_);
            if (visible)
            {
                ImGui::TextWrapped("Event CSV columns: simulationTimeS,type,message");
                ImGui::SetNextItemWidth(-1);
                ImGui::InputText("##eventPath", eventPath_, sizeof(eventPath_));
                if (ImGui::Button("Load events"))
                {
                    loadEvents(eventPath_);
                    showEventDialog_ = false;
                }
            }
            ImGui::End();
        }
#ifdef _WIN32
        if (showHardwareDialog_)
        {
            const bool visible = ImGui::Begin("Physical HIL setup (experimental)", &showHardwareDialog_);
            if (visible)
            {
                ImGui::TextWrapped("Use only with a known-safe bench setup. The ESP32 may require a manual reset after USB power reconnection; automatic recovery is not validated.");
                ImGui::Separator();
                ImGui::InputText("COM port", hilPort_, sizeof(hilPort_));
                ImGui::InputInt("Baud rate", &hilBaudRate_);
                ImGui::TextDisabled("Current tested configuration: COM3 / 460800 baud (after RST).");
                if (ImGui::Button("Start experimental hardware HIL"))
                {
                    startHilHardwareLive();
                    if (!notificationIsError_) showHardwareDialog_ = false;
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel")) showHardwareDialog_ = false;
            }
            ImGui::End();
        }
#endif
        drawStatusBar();
    }

    void VisualizationApp::shutdown() noexcept
    {
        stopLiveSource();
        if (window_) glfwMakeContextCurrent(window_);
        if (openglBackendInitialized_)
        {
            ImGui_ImplOpenGL3_Shutdown();
            openglBackendInitialized_ = false;
        }
        if (glfwBackendInitialized_)
        {
            ImGui_ImplGlfw_Shutdown();
            glfwBackendInitialized_ = false;
        }
        if (implotContextCreated_)
        {
            ImPlot::DestroyContext();
            implotContextCreated_ = false;
        }
        if (imguiContextCreated_)
        {
            ImGui::DestroyContext();
            imguiContextCreated_ = false;
        }
        if (window_)
        {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        if (glfwInitialized_)
        {
            glfwTerminate();
            glfwInitialized_ = false;
        }
    }
}
