#pragma once

#include "PanelVisibility.h"

#include <hil/HilTelemetryStore.h>
#include <visualization/Camera3D.h>
#include <visualization/FrameTimingStats.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSession.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

struct GLFWwindow;

namespace aeroforge::visualization::gui
{
    class VisualizationApp final
    {
    public:
        VisualizationApp() = default;
        ~VisualizationApp();
        VisualizationApp(const VisualizationApp&) = delete;
        VisualizationApp& operator=(const VisualizationApp&) = delete;

        bool initialize();
        int run();

    private:
        enum class SourceKind { None, Recording, Synthetic, SixDoF, SoftwareHIL, HardwareHIL };
        enum class WorkerStatus { Idle, Running, Completed, Failed };

        static void glfwErrorCallback(int code, const char* text);
        static void dropCallback(GLFWwindow* window, int count, const char** paths);

        void loadRecording(const char* path) noexcept;
        void loadEvents(const char* path) noexcept;
        void startSyntheticLive();
        void startRealLive();
        void startHilLive();
        void startHilHardwareLive();
        void stopSyntheticLive() noexcept;
        void stopRealLive() noexcept;
        void stopHilLive() noexcept;
        void stopLiveSource() noexcept;
        void failWorker(const std::string& description);
        std::string workerError() const;
        const char* sourceLabel() const noexcept;
        const char* statusLabel() const noexcept;

        void drawMenuBar();
        void drawDockspace();
        void drawStatusBar();
        void resetDefaultLayout(unsigned int id);
        void renderFrame();
        void shutdown() noexcept;

        GLFWwindow* window_ = nullptr;
        bool glfwInitialized_ = false;
        bool imguiContextCreated_ = false;
        bool implotContextCreated_ = false;
        bool glfwBackendInitialized_ = false;
        bool openglBackendInitialized_ = false;
        bool applyDefaultLayout_ = false;
        bool showImGuiDemo_ = false;
        bool showImPlotDemo_ = false;
        bool showLoadDialog_ = false;
        bool showEventDialog_ = false;
        bool showHardwareDialog_ = false;
        char filePath_[1024]{};
        char eventPath_[1024]{};
        char hilPort_[32] = "COM3";
        int hilBaudRate_ = 460800;
        std::string notification_;
        bool notificationIsError_ = false;
        SourceKind source_ = SourceKind::None;
        std::atomic<WorkerStatus> workerStatus_{ WorkerStatus::Idle };
        mutable std::mutex workerErrorMutex_;
        std::string workerError_;

        PanelVisibility panels_{};
        VisualizationSession session_;
        FrameTimingStats frameStats_;
        Camera3D camera_;

        std::shared_ptr<LiveTelemetryPublisher> demoPublisher_;
        std::jthread demoThread_;
        std::shared_ptr<LiveTelemetryPublisher> realPublisher_;
        std::jthread realThread_;
        // Keep the last HIL measurements visible after a run completes.
        std::shared_ptr<hil::HilTelemetryStore> hilTelemetry_;
        std::shared_ptr<LiveTelemetryPublisher> hilPublisher_;
        std::jthread hilThread_;
    };
}
