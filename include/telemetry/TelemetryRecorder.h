#pragma once

#include <telemetry/TelemetrySample.h>

#include <vector>
#include <cstddef>

namespace aeroforge::telemetry
{
	class TelemetryRecorder
	{
	public:
		void record(const TelemetrySample& sample);

		const std::vector<TelemetrySample>& getSamples() const;

		std::size_t size() const;

		bool empty() const;

		void clear();

	private:
		std::vector<TelemetrySample> samples;
	};
}