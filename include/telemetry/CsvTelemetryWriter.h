#pragma once

#include <telemetry/TelemetrySample.h>

#include <ostream>
#include <vector>

namespace aeroforge::telemetry
{
	class CsvTelemetryWriter
	{
	public:
		static void write(std::ostream& output, const std::vector<TelemetrySample>& samples);
	};
}