#include <visualization/Camera3D.h>
#include <visualization/PlotDataBuilder.h>
#include <visualization/TelemetryCsvLoader.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
using namespace aeroforge::visualization;
namespace
{
    VisualizationSnapshot example(double t)
    {
        using aeroforge::math::Quaternion;
        return { t,{t,std::sin(t),-t * 0.5},{1,0.1,-0.5},Quaternion::identity(),{0,0,0},
            {t + 0.1,std::sin(t),-t * 0.5},{1,0.1,-0.5},Quaternion::identity(),{0,0,0},
            {0,0,0},{0,0,0},{0,0,0} };
    }
    template<typename F> void measure(const char* name, int iterations, F&& work)
    {
        using clock = std::chrono::steady_clock;
        const auto start = clock::now();
        for (int i = 0;i < iterations;++i)work();
        const double seconds = std::chrono::duration<double>(clock::now() - start).count();
        std::cout << name << ',' << iterations << ',' << std::setprecision(12)
            << seconds << ',' << seconds * 1e6 / iterations << '\n';
    }
}
int main()
{
    // This is a repeatable runner, NOT a report of validated timings.
    std::vector<VisualizationSnapshot> samples;
    samples.reserve(50000);
    for (int i = 0;i < 50000;++i)samples.push_back(example(i * 0.01));
    std::stringstream csv;
    TelemetryCsvLoader::write(csv, samples);
    const std::string input = csv.str();
    std::cout << "benchmark,iterations,wall_seconds,mean_us_per_iteration\n";
    std::size_t consumed = 0;
    measure("CSV_parse_50000", 10, [&]
        {
            std::istringstream data(input);
            consumed += TelemetryCsvLoader::parse(data).size();
        });
    measure("Plot_decimate_50000", 300, [&]
        {consumed += PlotDataBuilder::decimate(samples, 2000, 0).size();});
    Camera3D camera;
    measure("Camera_projection_1000", 1000, [&]
        {
            for (int i = 0;i < 1000;++i)
                consumed += camera.project({ i * 0.001,0,0 }, 1280, 720).visible ? 1 : 0;
        });
    LiveTelemetryPublisher publisher;
    measure("Snapshot_publish_50000", 20, [&]
        {for (const auto& s : samples)publisher.publish(s);});
    std::cerr << "checksum=" << consumed << '\n';
}
