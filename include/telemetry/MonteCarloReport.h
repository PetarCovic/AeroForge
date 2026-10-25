#pragma once

#include <montecarlo/MonteCarloStatistics.h>
#include <montecarlo/MonteCarloTypes.h>

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace aeroforge::telemetry
{
	struct MonteCarloReportData
	{
		std::size_t trialCount;
		double successRate;
		std::map<std::string, montecarlo::MetricSummary> metricSummaries;
	};

	class MonteCarloReport
	{
	public:
		static MonteCarloReportData build(const std::vector <montecarlo::MonteCarloResult>& results, const std::vector<std::string>& metricNames);
	};
}