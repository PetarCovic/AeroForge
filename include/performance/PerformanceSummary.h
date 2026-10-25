#pragma once

#include <performance/BenchmarkResult.h>

#include <cstddef>
#include <string>
#include <vector>

namespace aeroforge::performance
{
	struct PerformanceSummaryData
	{
		std::string name;
		std::size_t benchmarkCount;
		double totalWallTimeS;
		double meanBenchmarkTimeS;
		double fastestBenchmarkTimeS;
		double slowestBenchmarkTimeS;
		std::size_t totalOperations;
	};

	class PerformanceSummary
	{
	public:
		static PerformanceSummaryData summarize(const std::string& name, const std::vector<BenchmarkResult>& results);
	};
}