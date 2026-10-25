#pragma once

#include <performance/BenchmarkResult.h>
#include <performance/RealtimeBenchmarkResult.h>

#include <vector>

namespace aeroforge::performance
{
	struct RealtimeBenchmarkSuiteResult
	{
		std::vector<BenchmarkResult> benchmarks;
		std::vector<RealtimeBenchmarkResult> realtimeMetrics;
	};
}