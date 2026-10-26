#include <visualization/VisualizationSession.h>
#include <visualization/TelemetryCsvLoader.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace aeroforge::visualization
{
    void VisualizationSession::loadRecording(const std::filesystem::path& path)
    {
        auto incoming = TelemetryCsvLoader::load(path);
        auto source = std::make_shared<RecordedTelemetryDataSource>(incoming);
        auto model = std::make_unique<VisualizationModel>(source);
        samples_ = std::move(incoming);
        recordedSource_ = std::move(source);
        recorded_ = std::move(model);
        live_.reset();
        events_.clear();
        sourceName_ = path.string();
        mode_ = Mode::Recorded;++revision_;
    }
    void VisualizationSession::loadEventCsv(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("Cannot open event CSV: " + path.string());
        // Separate event format: simulationTimeS,type,message. A message may contain
        // commas but must not contain embedded newlines; see user guide.
        std::string line;
        if (!std::getline(file, line) || line != "simulationTimeS,type,message")
            throw std::runtime_error("Event CSV header must equal simulationTimeS,type,message");
        std::vector<VisualEvent> loaded;
        std::size_t n = 1;
        while (std::getline(file, line))
        {
            ++n;
            if (line.empty())continue;
            const auto a = line.find(',');
            const auto b = line.find(',', a == std::string::npos ? a : a + 1);
            if (a == std::string::npos || b == std::string::npos)
                throw std::runtime_error("Malformed event at line " + std::to_string(n));
            const std::string ts = line.substr(0, a);
            std::size_t consumed = 0;
            double time = 0;
            try { time = std::stod(ts, &consumed); }
            catch (...) { throw std::runtime_error("Invalid event timestamp at line " + std::to_string(n)); }
            if (consumed != ts.size() || !std::isfinite(time) || time < 0)
                throw std::runtime_error("Invalid event timestamp at line " + std::to_string(n));
            loaded.push_back({ time,line.substr(a + 1,b - a - 1),line.substr(b + 1) });
        }
        std::stable_sort(loaded.begin(), loaded.end(), [](const auto& x, const auto& y)
            {return x.simulationTimeS < y.simulationTimeS;});
        events_ = std::move(loaded);
    }
    void VisualizationSession::attachLive(std::shared_ptr<const LiveTelemetryPublisher> publisher)
    {
        if (!publisher) throw std::invalid_argument("Live publisher is null");
        auto model = std::make_unique<LiveVisualizationModel>(std::move(publisher), liveCapacity_);
        samples_.clear();events_.clear();recorded_.reset();recordedSource_.reset();
        live_ = std::move(model);liveLastTime_ = -1;
        sourceName_ = "LiveTelemetryPublisher";mode_ = Mode::Live;++revision_;
    }
    void VisualizationSession::stopLive()
    {
        if (mode_ == Mode::Live) { live_.reset();samples_.clear();mode_ = Mode::None;sourceName_.clear();++revision_; }
    }
    void VisualizationSession::update(double wallDeltaS)
    {
        if (!std::isfinite(wallDeltaS) || wallDeltaS < 0)
            throw std::invalid_argument("Invalid wallDeltaS");
        if (mode_ == Mode::Recorded && recorded_)recorded_->update(wallDeltaS);
        if (mode_ == Mode::Live && live_)
        {
            live_->update();
            if (!live_->hasSnapshot())return;
            const auto& s = live_->getCurrentSnapshot();
            if (s.simulationTimeS == liveLastTime_)return;
            if (s.simulationTimeS < liveLastTime_)samples_.clear();
            liveLastTime_ = s.simulationTimeS;
            if (samples_.size() == liveCapacity_)samples_.erase(samples_.begin());
            samples_.push_back(s);++revision_;
        }
    }
    void VisualizationSession::play() { if (recorded_)recorded_->play(); }
    void VisualizationSession::pause() { if (recorded_)recorded_->pause(); }
    void VisualizationSession::restart() { if (recorded_)recorded_->restart(); }
    void VisualizationSession::seek(double time) { if (recorded_)recorded_->seek(time); }
    void VisualizationSession::setPlaybackRate(double rate)
    {
        if (recorded_)recorded_->setPlaybackRate(rate);
    }
    bool VisualizationSession::isPlaying() const
    {
        return mode_ == Mode::Recorded && recorded_ && recorded_->isPlaying();
    }
    double VisualizationSession::currentTimeS() const
    {
        if (mode_ == Mode::Recorded && recorded_)return recorded_->getCurrentTimeS();
        if (mode_ == Mode::Live && !samples_.empty())return samples_.back().simulationTimeS;
        return 0;
    }
    double VisualizationSession::startTimeS() const
    {
        return !samples_.empty() ? samples_.front().simulationTimeS : 0;
    }
    double VisualizationSession::endTimeS() const
    {
        return !samples_.empty() ? samples_.back().simulationTimeS : 0;
    }
    double VisualizationSession::playbackRate() const
    {
        return recorded_ ? recorded_->getPlaybackRate() : 1.0;
    }
    const VisualizationSnapshot* VisualizationSession::current() const
    {
        if (mode_ == Mode::Recorded && recorded_)return &recorded_->getCurrentSnapshot();
        if (mode_ == Mode::Live && live_ && live_->hasSnapshot())return &live_->getCurrentSnapshot();
        return nullptr;
    }
}
