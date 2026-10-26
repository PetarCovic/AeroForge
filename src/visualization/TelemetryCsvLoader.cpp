#include <visualization/TelemetryCsvLoader.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace aeroforge::visualization
{
    namespace
    {
        constexpr std::array<const char*, 36> fields = {
            "simulationTimeS",
            "truthPositionN.x", "truthPositionN.y", "truthPositionN.z",
            "truthVelocityN.x", "truthVelocityN.y", "truthVelocityN.z",
            "truthAttitudeNB.w", "truthAttitudeNB.x", "truthAttitudeNB.y", "truthAttitudeNB.z",
            "truthAngularVelocityB.x", "truthAngularVelocityB.y", "truthAngularVelocityB.z",
            "estimatedPositionN.x", "estimatedPositionN.y", "estimatedPositionN.z",
            "estimatedVelocityN.x", "estimatedVelocityN.y", "estimatedVelocityN.z",
            "estimatedAttitudeNB.w", "estimatedAttitudeNB.x", "estimatedAttitudeNB.y", "estimatedAttitudeNB.z",
            "estimatedAngularVelocityB.x", "estimatedAngularVelocityB.y", "estimatedAngularVelocityB.z",
            "attitudeErrorB.x", "attitudeErrorB.y", "attitudeErrorB.z",
            "commandedMomentB.x", "commandedMomentB.y", "commandedMomentB.z",
            "actualMomentB.x", "actualMomentB.y", "actualMomentB.z"
        };
        // Alternate names from telemetry::CsvTelemetryWriter (40-column format).
        // The final four actuator channels are not part of VisualizationSnapshot
        // and are intentionally ignored by this loader for now.
        constexpr std::array<const char*, 36> simulationWriterFields = {
            "time_s",
            "position_n_m", "position_e_m", "position_d_m",
            "velocity_n_mps", "velocity_e_mps", "velocity_d_mps",
            "attitude_q_w", "attitude_q_x", "attitude_q_y", "attitude_q_z",
            "omega_x_radps", "omega_y_radps", "omega_z_radps",
            "estimated_position_n_m", "estimated_position_e_m", "estimated_position_d_m",
            "estimated_velocity_n_mps", "estimated_velocity_e_mps", "estimated_velocity_d_mps",
            "estimated_attitude_q_w", "estimated_attitude_q_x", "estimated_attitude_q_y", "estimated_attitude_q_z",
            "estimated_omega_x_radps", "estimated_omega_y_radps", "estimated_omega_z_radps",
            "attitude_error_x_rad", "attitude_error_y_rad", "attitude_error_z_rad",
            "commanded_moment_x_nm", "commanded_moment_y_nm", "commanded_moment_z_nm",
            "actual_moment_x_nm", "actual_moment_y_nm", "actual_moment_z_nm"
        };
        std::string normalized(std::string_view text)
        {
            std::string out;
            for (const unsigned char c : text)
                if (std::isalnum(c)) out.push_back(static_cast<char>(std::tolower(c)));
            return out;
        }
        std::string trim(std::string s)
        {
            const auto first = s.find_first_not_of(" \t\r\n");
            if (first == std::string::npos) return {};
            const auto last = s.find_last_not_of(" \t\r\n");
            return s.substr(first, last - first + 1);
        }
        std::vector<std::string> cells(const std::string& line)
        {
            std::vector<std::string> result;
            std::string value;
            bool quoted = false;
            for (std::size_t i = 0; i < line.size(); ++i)
            {
                const char c = line[i];
                if (c == '"')
                {
                    if (quoted && i + 1 < line.size() && line[i + 1] == '"')
                    {
                        value += '"'; ++i;
                    }
                    else quoted = !quoted;
                }
                else if (c == ',' && !quoted)
                {
                    result.push_back(trim(value)); value.clear();
                }
                else value += c;
            }
            if (quoted) throw std::runtime_error("Unterminated quoted CSV field");
            result.push_back(trim(value));
            return result;
        }
        double number(const std::string& value, std::size_t row, const char* column)
        {
            std::size_t used = 0;
            double result = 0.0;
            try { result = std::stod(value, &used); }
            catch (const std::exception&) {
                throw std::runtime_error("CSV line " + std::to_string(row) +
                    ": invalid " + column);
            }
            if (used != value.size() || !std::isfinite(result))
                throw std::runtime_error("CSV line " + std::to_string(row) +
                    ": non-finite or invalid " + column);
            return result;
        }
        math::Vector3 vec(const std::array<double, 36>& a, std::size_t i)
        {
            return { a[i], a[i + 1], a[i + 2] };
        }
        math::Quaternion quat(const std::array<double, 36>& a, std::size_t i, std::size_t line)
        {
            math::Quaternion q(a[i], a[i + 1], a[i + 2], a[i + 3]);
            if (!q.isFinite() || !std::isfinite(q.squaredNorm()) || q.squaredNorm() < 1e-20)
                throw std::runtime_error("CSV line " + std::to_string(line) + ": invalid quaternion");
            return q.normalized();
        }
        void emitVec(std::ostream& os, const math::Vector3& v)
        {
            os << ',' << v.x << ',' << v.y << ',' << v.z;
        }
        void emitQuat(std::ostream& os, const math::Quaternion& q)
        {
            os << ',' << q.w << ',' << q.x << ',' << q.y << ',' << q.z;
        }
    }

    std::vector<VisualizationSnapshot> TelemetryCsvLoader::load(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("Unable to open telemetry file: " + path.string());
        return parse(file);
    }

    std::vector<VisualizationSnapshot> TelemetryCsvLoader::parse(std::istream& input)
    {
        std::string header;
        if (!std::getline(input, header)) throw std::runtime_error("Telemetry CSV is empty");
        if (header.size() >= 3 &&
            static_cast<unsigned char>(header[0]) == 0xEF &&
            static_cast<unsigned char>(header[1]) == 0xBB &&
            static_cast<unsigned char>(header[2]) == 0xBF)
            header.erase(0, 3);
        const auto headings = cells(header);
        std::map<std::string, std::size_t> columns;
        for (std::size_t i = 0; i < headings.size(); ++i)
        {
            const auto key = normalized(headings[i]);
            if (key.empty() || !columns.emplace(key, i).second)
                throw std::runtime_error("Empty or duplicate CSV column: " + headings[i]);
        }
        std::array<int, 36> index{};
        index.fill(-1);
        for (std::size_t i = 0; i < fields.size(); ++i)
        {
            auto it = columns.find(normalized(fields[i]));
            if (it == columns.end())
                it = columns.find(normalized(simulationWriterFields[i]));
            if (it != columns.end()) index[i] = static_cast<int>(it->second);
        }
        // The truthful minimum is time, NED truth position/velocity and body->NED attitude.
        for (std::size_t i = 0;i <= 10;++i)
            if (index[i] < 0) throw std::runtime_error(std::string("Required CSV column absent: ") + fields[i]);
        // Optional groups must be complete, preventing half-mapped vectors/quaternions.
        const std::array<std::pair<int, int>, 6> groups = { {{11,13},{14,16},{17,19},{20,23},{24,26},{27,29}} };
        for (const auto& [lo, hi] : groups)
        {
            int count = 0;for (int i = lo;i <= hi;++i) count += (index[i] >= 0);
            if (count != 0 && count != hi - lo + 1)
                throw std::runtime_error(std::string("Incomplete CSV field group starting: ") + fields[lo]);
        }
        for (int lo = 30;lo <= 33;lo += 3)
        {
            int count = 0;for (int i = lo;i < lo + 3;++i) count += (index[i] >= 0);
            if (count != 0 && count != 3)
                throw std::runtime_error(std::string("Incomplete CSV field group starting: ") + fields[lo]);
        }
        std::vector<VisualizationSnapshot> samples;
        std::string line;
        std::size_t lineNo = 1;
        double previousTime = -1.0;
        while (std::getline(input, line))
        {
            ++lineNo;
            if (trim(line).empty()) continue;
            const auto values = cells(line);
            if (values.size() != headings.size())
                throw std::runtime_error("CSV line " + std::to_string(lineNo) + ": column count mismatch");
            std::array<double, 36> data{};
            for (std::size_t i = 0;i < data.size();++i)
                if (index[i] >= 0) data[i] = number(values[static_cast<std::size_t>(index[i])], lineNo, fields[i]);
            if (data[0] < 0 || data[0] <= previousTime)
                throw std::runtime_error("CSV line " + std::to_string(lineNo) + ": times must increase from a nonnegative value");
            previousTime = data[0];
            VisualizationSnapshot s{
                data[0],vec(data,1),vec(data,4),quat(data,7,lineNo),vec(data,11),
                index[14] >= 0 ? vec(data,14) : vec(data,1),
                index[17] >= 0 ? vec(data,17) : vec(data,4),
                index[20] >= 0 ? quat(data,20,lineNo) : quat(data,7,lineNo),
                index[24] >= 0 ? vec(data,24) : vec(data,11),
                vec(data,27),vec(data,30),vec(data,33)
            };
            samples.push_back(s);
        }
        if (samples.empty()) throw std::runtime_error("Telemetry CSV has no samples");
        return samples;
    }

    void TelemetryCsvLoader::write(std::ostream& output,
        const std::vector<VisualizationSnapshot>& samples)
    {
        for (std::size_t i = 0;i < fields.size();++i)
            output << (i == 0 ? "" : ",") << fields[i];
        output << '\n' << std::setprecision(17);
        for (const auto& s : samples)
        {
            output << s.simulationTimeS;
            emitVec(output, s.truthPositionN);emitVec(output, s.truthVelocityN);
            emitQuat(output, s.truthAttitudeNB);emitVec(output, s.truthAngularVelocityB);
            emitVec(output, s.estimatedPositionN);emitVec(output, s.estimatedVelocityN);
            emitQuat(output, s.estimatedAttitudeNB);emitVec(output, s.estimatedAngularVelocityB);
            emitVec(output, s.attitudeErrorB);emitVec(output, s.commandedMomentB);
            emitVec(output, s.actualMomentB);
            output << '\n';
        }
    }
}
