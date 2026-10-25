#include <montecarlo/MonteCarloStatistics.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::montecarlo
{
	MetricSummary MonteCarloStatistics::summarizeMetric(const std::vector<MonteCarloResult>& results, const std::string& metricName)
	{
		if (metricName.empty())
		{
			throw std::invalid_argument("metricName cannot be empty");
		}

		std::vector<double> values;
		double sum = 0;

		for (std::size_t i = 0; i < results.size(); i++)
		{
			if (!results[i].metrics.contains(metricName))
			{
				continue;
			}

			double value = results[i].metrics.at(metricName);

			if (!std::isfinite(value))
			{
				throw std::invalid_argument("value must be finite");
			}

			sum += value;
			values.push_back(value);
		}

		if (values.empty())
		{
			throw std::invalid_argument("values cannot be empty");
		}

		double mean = sum / values.size();
		double squaredDeviationSum = 0;

		for (double value : values)
		{
			double deviation = value - mean;
			double squaredDeviation = deviation * deviation;

			squaredDeviationSum += squaredDeviation;
		}

		double variance = squaredDeviationSum / values.size();
		double standardDeviation = std::sqrt(variance);

		double min = *std::min_element(values.begin(), values.end());
		double max = *std::max_element(values.begin(), values.end());

		MetricSummary summary;
		summary.sampleCount=values.size();
		summary.mean=mean;
		summary.standardDeviation=standardDeviation;
		summary.min=min;
		summary.max=max;

		return summary;
	}

	double MonteCarloStatistics::computeSuccessRate(const std::vector<MonteCarloResult>& results)
	{
		if (results.empty())
		{
			throw std::invalid_argument("results cannot be empty");
		}

		std::size_t successCount = 0;
		std::size_t totalResults = results.size();
		for (std::size_t i = 0; i < totalResults; i++)
		{
			if (results[i].success)
			{
				successCount++;
			}
		}

		return static_cast<double>(successCount)/totalResults;
	}
}