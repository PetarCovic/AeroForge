#pragma once
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/LiveVisualizationModel.h>
#include <visualization/RecordedTelemetryDataSource.h>
#include <visualization/VisualizationModel.h>
#include <visualization/VisualizationSnapshot.h>
#include <memory>
#include <optional>
#include <filesystem>
#include <string>
#include <vector>

namespace aeroforge::visualization
{
    struct VisualEvent
    {
        double simulationTimeS = 0;
        std::string type;
        std::string message;
    };
    class VisualizationSession
    {
    public:
        enum class Mode { None, Recorded, Live };
        void loadRecording(const std::filesystem::path& path);
        void loadEventCsv(const std::filesystem::path& path);
        void attachLive(std::shared_ptr<const LiveTelemetryPublisher> publisher);
        void stopLive();
        void update(double wallDeltaS);
        void play();
        void pause();
        void restart();
        void seek(double simulationTimeS);
        void setPlaybackRate(double rate);
        bool isPlaying() const;
        double currentTimeS() const;
        double startTimeS() const;
        double endTimeS() const;
        double playbackRate() const;
        const VisualizationSnapshot* current() const;
        const std::vector<VisualizationSnapshot>& samples() const { return samples_; }
        const std::vector<VisualEvent>& events() const { return events_; }
        Mode mode() const { return mode_; }
        std::size_t revision() const { return revision_; }
        const std::string& sourceName() const { return sourceName_; }
    private:
        Mode mode_ = Mode::None;
        std::size_t revision_ = 0;
        std::string sourceName_;
        std::vector<VisualizationSnapshot> samples_;
        std::vector<VisualEvent> events_;
        std::shared_ptr<RecordedTelemetryDataSource> recordedSource_;
        std::unique_ptr<VisualizationModel> recorded_;
        std::unique_ptr<LiveVisualizationModel> live_;
        double liveLastTime_ = -1;
        static constexpr std::size_t liveCapacity_ = 2048;
    };
}
