#include <telemetry/MonteCarloReport.h>

#include <stdexcept>

namespace aeroforge::telemetry
{
	MonteCarloReportData MonteCarloReport::build(const std::vector <montecarlo::MonteCarloResult>& results, const std::vector<std::string>& metricNames)
	{
		if (results.empty())
		{
			throw std::invalid_argument("results cannot be empty");
		}

		for (const std::string& metricName : metricNames)
		{
			if (metricName.empty())
			{
				throw std::invalid_argument("metricName cannot be empty");
			}
		}

		MonteCarloReportData reportData;
		reportData.trialCount = results.size();
		reportData.successRate = montecarlo::MonteCarloStatistics::computeSuccessRate(results);

		for (const std::string& metricName : metricNames)
		{
			reportData.metricSummaries.emplace(metricName, montecarlo::MonteCarloStatistics::summarizeMetric(results, metricName));
		}

		return reportData;
	}
}