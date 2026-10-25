#include <performance/PerformanceSummary.h>

#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace aeroforge::performance
{
	PerformanceSummaryData PerformanceSummary::summarize(const std::string& name, const std::vector<BenchmarkResult>& results)
	{
		if (name.empty())
		{
			throw std::invalid_argument("name cannot be empty");
		}

		if (results.empty())
		{
			throw std::invalid_argument("results cannot be empty");
		}

		double wallTimeSSum=0.0;
		std::size_t iterationSum=0;
		double minTotalTimeS = results[0].totalTimeS;
		double maxTotalTimeS = results[0].totalTimeS;

		for (const BenchmarkResult& result : results)
		{
			if (result.iterations<=0)
			{
				throw std::invalid_argument("result.iterations must be positive");
			}

			if (!std::isfinite(result.totalTimeS) || result.totalTimeS<0)
			{
				throw std::invalid_argument("result.totalTimeS must be finite and non-negative");
			}

			if (!std::isfinite(result.meanTimeS) || result.meanTimeS < 0)
			{
				throw std::invalid_argument("result.meanTimeS must be finite and non-negative");
			}

			if (!std::isfinite(result.minTimeS) || result.minTimeS < 0)
			{
				throw std::invalid_argument("result.minTimeS must be finite and non-negative");
			}

			if (!std::isfinite(result.maxTimeS) || result.maxTimeS < 0)
			{
				throw std::invalid_argument("result.maxTimeS must be finite and non-negative");
			}

			if (!std::isfinite(result.operationsPerSecond) || result.operationsPerSecond < 0)
			{
				throw std::invalid_argument("result.operationsPerSecond must be finite and non-negative");
			}

			wallTimeSSum += result.totalTimeS;
			iterationSum += result.iterations;
			minTotalTimeS = std::min(minTotalTimeS, result.totalTimeS);
			maxTotalTimeS = std::max(maxTotalTimeS, result.totalTimeS);
		}

		PerformanceSummaryData summaryData;
		summaryData.name = name;
		summaryData.benchmarkCount = results.size();
		summaryData.totalWallTimeS = wallTimeSSum;
		summaryData.totalOperations = iterationSum;
		summaryData.meanBenchmarkTimeS = summaryData.totalWallTimeS / summaryData.benchmarkCount;
		summaryData.fastestBenchmarkTimeS = minTotalTimeS;
		summaryData.slowestBenchmarkTimeS = maxTotalTimeS;

		return summaryData;
	}
}