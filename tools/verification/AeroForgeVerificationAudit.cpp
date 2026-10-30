// Audit the machine-generated HIL benchmark artifacts without repeating
// the simulation's existing GoogleTests.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    namespace fs = std::filesystem;

    enum class Mode { Software, Hardware };

    struct Evidence
    {
        std::uint64_t rows = 0;
        std::uint64_t finalAccepted = 0;
        std::uint64_t finalRejected = 0;
        std::uint64_t finalDropped = 0;
        std::uint64_t executionMisses = 0;
        bool executionMissesInCsv = false;
        std::uint64_t completionMisses = 0;
        std::uint64_t validSteps = 0;
        std::uint64_t zeroFallbackSteps = 0;
        double periodS = 0.0;
        double lastSimulationTimeS = 0.0;
        double maxExecutionMs = 0.0;
        double meanExecutionMs = 0.0;
        double p95ExecutionMs = 0.0;
        double p99ExecutionMs = 0.0;
        double maxStartLatenessMs = 0.0;
        double p99StartLatenessMs = 0.0;
        std::vector<std::string> warnings;
    };

    [[noreturn]] void error(const std::string& message)
    {
        throw std::runtime_error(message);
    }

    std::string stripCR(std::string s)
    {
        if (!s.empty() && s.back() == '\r') s.pop_back();
        return s;
    }

    std::vector<std::string> splitCsv(const std::string& line)
    {
        // AeroForge's HIL benchmark CSV uses unquoted numeric fields.
        std::vector<std::string> result;
        std::size_t begin = 0;
        while (true)
        {
            const auto comma = line.find(',', begin);
            result.push_back(line.substr(begin, comma - begin));
            if (comma == std::string::npos) break;
            begin = comma + 1;
        }
        return result;
    }

    double number(const std::string& s, const std::string& field)
    {
        std::size_t consumed = 0;
        double v = 0.0;
        try { v = std::stod(s, &consumed); }
        catch (const std::exception&) { error("Invalid number in " + field + ": " + s); }
        if (consumed != s.size() || !std::isfinite(v))
            error("Non-finite or invalid number in " + field + ": " + s);
        return v;
    }

    std::uint64_t count(const std::string& s, const std::string& field)
    {
        const double v = number(s, field);
        if (v < 0 || v >= 9007199254740992.0 || std::floor(v) != v)
            error("Invalid nonnegative integer in " + field);
        return static_cast<std::uint64_t>(v);
    }

    double percentile(std::vector<double> values, double fraction)
    {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const auto rank = static_cast<std::size_t>(
            std::ceil(fraction * static_cast<double>(values.size())));
        return values[std::max<std::size_t>(1, rank) - 1];
    }

    class Row
    {
    public:
        Row(const std::map<std::string, std::size_t>& headings,
            const std::vector<std::string>& cells)
            : headings_(headings), cells_(cells) {
        }

        double n(const std::string& key) const
        {
            const auto it = headings_.find(key);
            if (it == headings_.end()) error("Missing CSV column: " + key);
            if (it->second >= cells_.size()) error("Missing CSV value: " + key);
            return number(cells_[it->second], key);
        }

        std::uint64_t u(const std::string& key) const
        {
            const auto it = headings_.find(key);
            if (it == headings_.end()) error("Missing CSV column: " + key);
            if (it->second >= cells_.size()) error("Missing CSV value: " + key);
            return count(cells_[it->second], key);
        }

    private:
        const std::map<std::string, std::size_t>& headings_;
        const std::vector<std::string>& cells_;
    };

    Evidence auditCsv(Mode mode, const fs::path& path)
    {
        std::ifstream in(path);
        if (!in) error("Cannot open benchmark CSV: " + path.string());

        std::string line;
        if (!std::getline(in, line)) error("Empty benchmark CSV");
        const auto header = splitCsv(stripCR(line));
        std::map<std::string, std::size_t> headings;
        for (std::size_t i = 0; i < header.size(); ++i)
            if (!headings.emplace(header[i], i).second)
                error("Duplicate CSV column: " + header[i]);

        Evidence e;
        e.executionMissesInCsv = headings.contains("deadline_misses");
        std::vector<double> execution;
        std::vector<double> lateness;
        std::uint64_t previousAccepted = 0;
        std::uint64_t previousRejected = 0;
        std::uint64_t previousDropped = 0;
        double previousTime = 0.0;

        while (std::getline(in, line))
        {
            line = stripCR(line);
            if (line.empty()) continue;
            const auto cells = splitCsv(line);
            if (cells.size() != header.size())
                error("CSV column count mismatch on data row " +
                    std::to_string(e.rows + 1));

            const Row row(headings, cells);
            const double t = row.n("simulation_time_s");
            const double stepMs = row.n("step_execution_ms");
            const double commMs = row.n("communication_ms");
            const double latencyMs = row.n("new_command_latency_ms");
            const auto accepted = row.u("accepted_commands");
            const auto rejected = row.u("rejected_commands");
            const std::uint64_t dropped = row.u("dropped_samples");

            if (stepMs < 0 || commMs < 0 || latencyMs < -1.0)
                error("Invalid measured timing in CSV row " +
                    std::to_string(e.rows + 1));
            if (accepted < previousAccepted || rejected < previousRejected ||
                dropped < previousDropped)
                error("Nonmonotonic HIL counters in CSV row " +
                    std::to_string(e.rows + 1));

            if (e.rows == 0)
            {
                if (t <= 0) error("First simulation timestamp must be positive");
                e.periodS = t;
            }
            else if (t <= previousTime)
                error("Simulation timestamps are not strictly increasing");

            const double expectedT =
                static_cast<double>(e.rows + 1) * e.periodS;
            const double tolerance = 1e-6 + 1e-6 * expectedT;
            if (std::abs(t - expectedT) > tolerance)
                error("Simulation time differs from fixed-step schedule at row " +
                    std::to_string(e.rows + 1));

            execution.push_back(stepMs);
            e.maxExecutionMs = std::max(e.maxExecutionMs, stepMs);

            if (mode == Mode::Hardware)
            {
                const auto valid = row.u("used_valid_command");
                const auto miss = row.u("completion_deadline_miss");
                const double startLate = row.n("start_lateness_ms");
                const double overrun = row.n("completion_overrun_ms");
                if (valid > 1 || miss > 1 || startLate < 0 || overrun < 0)
                    error("Invalid hardware status field in CSV");
                e.validSteps += valid;
                e.zeroFallbackSteps += (1 - valid);
                e.completionMisses += miss;
                lateness.push_back(startLate);
                e.maxStartLatenessMs = std::max(e.maxStartLatenessMs, startLate);
                if (miss == 0 && overrun > 0)
                    error("Completion overrun without deadline miss");
            }
            else if (e.executionMissesInCsv)
            {
                const auto misses = row.u("deadline_misses");
                if (misses < e.executionMisses)
                    error("Nonmonotonic execution deadline miss count");
                e.executionMisses = misses;
            }

            e.finalAccepted = previousAccepted = accepted;
            e.finalRejected = previousRejected = rejected;
            e.finalDropped = previousDropped = dropped;
            previousTime = t;
            ++e.rows;
        }

        if (!in.eof() && in.fail()) error("Error reading benchmark CSV");
        if (e.rows == 0) error("Benchmark CSV contains no data rows");
        e.lastSimulationTimeS = previousTime;
        e.meanExecutionMs = std::accumulate(
            execution.begin(), execution.end(), 0.0) /
            static_cast<double>(execution.size());
        e.p95ExecutionMs = percentile(execution, 0.95);
        e.p99ExecutionMs = percentile(execution, 0.99);
        e.p99StartLatenessMs = percentile(lateness, 0.99);
        return e;
    }

    std::string readText(const fs::path& path)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in) error("Cannot open benchmark summary: " + path.string());
        std::ostringstream out;
        out << in.rdbuf();
        if (!in.good() && !in.eof()) error("Error reading JSON summary");
        const std::string text = out.str();
        const auto first = text.find_first_not_of(" \t\r\n");
        const auto last = text.find_last_not_of(" \t\r\n");
        if (first == std::string::npos || text[first] != '{' || text[last] != '}')
            error("Expected a JSON object in benchmark summary");
        return text;
    }

    double jsonNumber(const std::string& json, const std::string& key)
    {
        // The AeroForge benchmark summaries are flat scalar JSON objects.
        // Parse their known numeric fields; do not interpret arbitrary JSON.
        const std::regex field("\\\"" + key +
            "\\\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
        std::smatch match;
        if (!std::regex_search(json, match, field))
            error("Missing numeric JSON key: " + key);
        return number(match[1].str(), key);
    }

    std::uint64_t jsonCount(const std::string& json, const std::string& key)
    {
        return count(std::to_string(jsonNumber(json, key)), key);
    }

    bool jsonBool(const std::string& json, const std::string& key)
    {
        const std::regex field("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
        std::smatch match;
        if (!std::regex_search(json, match, field))
            error("Missing boolean JSON key: " + key);
        return match[1].str() == "true";
    }

    void requireMatch(std::uint64_t a, std::uint64_t b,
        const std::string& label)
    {
        if (a != b) error("CSV/JSON mismatch in " + label + ": " +
            std::to_string(a) + " vs " + std::to_string(b));
    }

    void auditSummary(Mode mode, Evidence& e, const fs::path& path)
    {
        const auto json = readText(path);
        requireMatch(e.rows, jsonCount(json, "completed_steps"), "completed_steps");
        requireMatch(e.finalAccepted, jsonCount(json, "accepted_commands"),
            "accepted_commands");
        requireMatch(e.finalRejected, jsonCount(json, "rejected_commands"),
            "rejected_commands");
        requireMatch(e.finalDropped, jsonCount(json, "dropped_sensor_samples"),
            "dropped_sensor_samples");

        const double summaryPeriodMs = jsonNumber(json, "step_period_ms");
        if (summaryPeriodMs <= 0 ||
            std::abs(summaryPeriodMs - e.periodS * 1000.0) > 1e-4)
            error("CSV/JSON mismatch in step period");

        const auto summaryExecutionMisses =
            jsonCount(json, "step_execution_deadline_misses");
        if (mode == Mode::Software)
        {
            if (e.executionMissesInCsv)
            {
                requireMatch(e.executionMisses, summaryExecutionMisses,
                    "step_execution_deadline_misses");
            }
            else
            {
                // Older/newer software benchmarks may omit a per-row
                // cumulative miss counter. The summary still records it.
                e.executionMisses = summaryExecutionMisses;
                e.warnings.emplace_back(
                    "Software CSV has no deadline_misses column; "
                    "execution misses were read from JSON only");
            }
        }
        else
        {
            requireMatch(e.validSteps,
                jsonCount(json, "steps_with_valid_command"),
                "steps_with_valid_command");
            requireMatch(e.zeroFallbackSteps,
                jsonCount(json, "steps_using_zero_fallback"),
                "steps_using_zero_fallback");
            requireMatch(e.completionMisses,
                jsonCount(json, "completion_deadline_misses"),
                "completion_deadline_misses");
            if (jsonBool(json, "serial_failed"))
                e.warnings.emplace_back("Hardware serial transport reported failure");
            if (jsonBool(json, "cancelled"))
                e.warnings.emplace_back("Hardware run was cancelled");
            if (jsonCount(json, "serial_rx_dropped_bytes") != 0)
                e.warnings.emplace_back("UART receive buffer dropped bytes");
            if (jsonCount(json, "serial_last_error") != 0)
                e.warnings.emplace_back("Windows serial error was reported");
            if (e.validSteps * 10 < e.rows * 9)
                e.warnings.emplace_back("Fewer than 90% of steps had fresh-enough control");
            if (e.finalAccepted * 10 < e.rows * 9)
                e.warnings.emplace_back(
                    "Controller response rate below 90% of simulation step rate; "
                    "this is a performance observation, not a corrupt evidence file");
        }

        const double reportedMean = jsonNumber(json, "mean_step_execution_ms");
        const double meanTolerance = std::max(0.002, 0.02 * reportedMean);
        if (std::abs(reportedMean - e.meanExecutionMs) > meanTolerance)
            error("CSV/JSON mean execution time does not agree");

        if (summaryExecutionMisses != 0)
            e.warnings.emplace_back("Nonzero execution deadline misses");
        if (e.completionMisses != 0)
            e.warnings.emplace_back("Nonzero completion deadline misses");
    }

    void writeReport(Mode mode, const Evidence& e, const fs::path& path)
    {
        if (path.has_parent_path())
            fs::create_directories(path.parent_path());
        std::ofstream report(path, std::ios::trunc);
        if (!report) error("Cannot create report: " + path.string());
        report << std::fixed << std::setprecision(6);
        report << "# AeroForge HIL Evidence Audit\n\n"
            << "- Mode: " << (mode == Mode::Hardware ? "Physical ESP32" : "Software loopback") << "\n"
            << "- Structural evidence audit: **PASS**\n"
            << "- Completed steps / CSV rows: " << e.rows << "\n"
            << "- Step period: " << e.periodS * 1000.0 << " ms\n"
            << "- Simulated duration: " << e.lastSimulationTimeS << " s\n"
            << "- Accepted commands: " << e.finalAccepted << "\n"
            << "- Rejected commands: " << e.finalRejected << "\n"
            << "- Dropped sensor samples: " << e.finalDropped << "\n"
            << "- Mean/p95/p99/max execution: " << e.meanExecutionMs << " / "
            << e.p95ExecutionMs << " / " << e.p99ExecutionMs << " / "
            << e.maxExecutionMs << " ms\n";
        if (mode == Mode::Hardware)
        {
            report << "- Steps with valid commands: " << e.validSteps << "\n"
                << "- Zero-command fallback steps: " << e.zeroFallbackSteps << "\n"
                << "- Completion deadline misses: " << e.completionMisses << "\n"
                << "- p99/max start lateness: " << e.p99StartLatenessMs
                << " / " << e.maxStartLatenessMs << " ms\n";
        }
        else
            report << "- Execution deadline misses: " << e.executionMisses << "\n";
        report << "\n## Observations\n\n";
        if (e.warnings.empty()) report << "No additional warnings from this audit.\n";
        for (const auto& warning : e.warnings)
            report << "- " << warning << "\n";
        report << "\nThis checks artifact consistency. It does not replace "
            "GoogleTests, numerical reference validation, physical "
            "disconnect testing, or hard real-time certification.\n";
        report.close();
        if (!report) error("Failed to write report");
    }
}

int main(int argc, char** argv)
{
    try
    {
        if (argc != 5)
        {
            std::cerr << "Usage: AeroForgeVerificationAudit <software|hardware> "
                "<samples.csv> <summary.json> <report.md>\n";
            return 2;
        }
        const std::string modeName = argv[1];
        if (modeName != "software" && modeName != "hardware")
            error("Mode must be software or hardware");
        const Mode mode = modeName == "hardware" ? Mode::Hardware : Mode::Software;
        auto evidence = auditCsv(mode, argv[2]);
        auditSummary(mode, evidence, argv[3]);
        writeReport(mode, evidence, argv[4]);
        std::cout << "HIL evidence audit PASSED (" << evidence.rows
            << " steps, " << evidence.finalAccepted << " commands, "
            << evidence.warnings.size() << " warning(s))\n";
        for (const auto& w : evidence.warnings)
            std::cout << "WARNING: " << w << '\n';
        std::cout << "Report: " << argv[4] << '\n';
        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "HIL evidence audit FAILED: " << ex.what() << '\n';
        return 1;
    }
}
