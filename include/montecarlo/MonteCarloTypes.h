#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace aeroforge::montecarlo
{
	struct MonteCarloTrial
	{
		std::size_t trialIndex;
		std::uint32_t seed;
		std::map<std::string, double> parameters;
	};

	struct MonteCarloResult
	{
		std::size_t trialIndex;
		bool success;
		std::map<std::string, double> metrics;
	};

	struct MetricSummary
	{
		std::size_t sampleCount;
		double mean;
		double standardDeviation;
		double min;
		double max;
	};
}