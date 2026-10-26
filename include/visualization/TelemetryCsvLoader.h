#pragma once
#include <visualization/VisualizationSnapshot.h>
#include <filesystem>
#include <iosfwd>
#include <vector>

namespace aeroforge::visualization
{
    // Canonical visualization snapshot CSV; the simulation writer's schema
    // must be mapped separately if its headers do not match these names.
    class TelemetryCsvLoader
    {
    public:
        static std::vector<VisualizationSnapshot> load(const std::filesystem::path& path);
        static std::vector<VisualizationSnapshot> parse(std::istream& input);
        static void write(std::ostream& output, const std::vector<VisualizationSnapshot>& samples);
    };
}
