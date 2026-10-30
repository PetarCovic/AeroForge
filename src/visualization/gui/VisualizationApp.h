#pragma once

#include "PanelVisibility.h"

#include <hil/HilTelemetryStore.h>
#include <visualization/Camera3D.h>
#include <visualization/FrameTimingStats.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSession.h>

#include <memory>
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
        static void glfwErrorCallback(int code, const char* text);
        static void dropCallback(GLFWwindow* window, int count,
            const char** paths);

        void loadRecording(const char* path) noexcept;
        void loadEvents(const char* path) noexcept;
        void startSyntheticLive();
        void stopSyntheticLive() noexcept;
        void startRealLive();
        void stopRealLive() noexcept;
        void startHilLive();
        void startHilHardwareLive();
        void stopHilLive() noexcept;
        void drawMenuBar();
        void drawDockspace();
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
        char filePath_[1024]{};
        char eventPath_[1024]{};
        std::string notification_;

        PanelVisibility panels_{};
        VisualizationSession session_;
        FrameTimingStats frameStats_;
        Camera3D camera_;

        std::shared_ptr<LiveTelemetryPublisher> demoPublisher_;
        std::jthread demoThread_;
        std::shared_ptr<LiveTelemetryPublisher> realPublisher_;
        std::jthread realThread_;

        // One HIL worker at a time: software or physical ESP32.
        std::shared_ptr<hil::HilTelemetryStore> hilTelemetry_;
        std::shared_ptr<LiveTelemetryPublisher> hilPublisher_;
        std::jthread hilThread_;
    };
}
