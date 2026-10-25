#pragma once

#include <performance/BenchmarkResult.h>
#include <performance/RealtimeBenchmarkSuiteResult.h>

#include <vector>

namespace aeroforge::performance
{
	std::vector<BenchmarkResult> runMathBenchmarks();

	std::vector<BenchmarkResult> runIntegratorBenchmarks();

	std::vector<BenchmarkResult> runSixDoFBenchmarks();

	std::vector<BenchmarkResult> runGncBenchmarks();

	std::vector<BenchmarkResult> runTelemetryBenchmarks();

	std::vector<BenchmarkResult> runMonteCarloBenchmarks();

	RealtimeBenchmarkSuiteResult runRealtimeBenchmarks();
}