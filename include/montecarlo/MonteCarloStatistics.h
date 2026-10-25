#pragma once

#include <montecarlo/MonteCarloTypes.h>

#include <string>
#include <vector>

namespace aeroforge::montecarlo
{
	class MonteCarloStatistics
	{
	public:
		static MetricSummary summarizeMetric(const std::vector<MonteCarloResult>& results, const std::string& metricName);

		static double computeSuccessRate(const std::vector<MonteCarloResult>& results);
	};
}