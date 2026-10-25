#include "performance/BenchmarkSuites.h"

#include <performance/PerformanceSummary.h>

#include <iostream>
#include <vector>
#include <exception>

namespace
{
	void printBenchmarkResult(const aeroforge::performance::BenchmarkResult& result)
	{
		std::cout
			<< "Benchmark: " << result.name << '\n'
			<< "Iterations: " << result.iterations << '\n'
			<< "Total Time: " << result.totalTimeS << " s\n"
			<< "Mean Time: " << result.meanTimeS * 1'000'000.0 << " us\n"
			<< "Operations/sec: " << result.operationsPerSecond << '\n'
			<< "Min Time: " << result.minTimeS * 1'000'000.0 << " us\n"
			<< "Max Time: " << result.maxTimeS * 1'000'000.0 << " us\n";
	}

	void printSummary(const aeroforge::performance::PerformanceSummaryData& summary)
	{
		std::cout
			<< "Suite: " << summary.name << '\n'
			<< "Benchmark count: " << summary.benchmarkCount << '\n'
			<< "Total operations: " << summary.totalOperations << '\n'
			<< "Total wall time: " << summary.totalWallTimeS * 1'000'000.0 << " us\n"
			<< "Mean benchmark wall time: " << summary.meanBenchmarkTimeS * 1'000'000.0 << " us\n"
			<< "Fastest benchmark time: " << summary.fastestBenchmarkTimeS * 1'000'000.0 << " us\n"
			<< "Slowest benchmark time: " << summary.slowestBenchmarkTimeS * 1'000'000.0 << " us\n";
	}

	void printRealtimeBenchmarkResult(const aeroforge::performance::RealtimeBenchmarkResult& result)
	{
		std::cout
			<< "  " << result.name << '\n'
			<< "    Simulated time:      "
			<< result.simulatedTimeS << " s\n"
			<< "    Wall time:           "
			<< result.wallTimeS << " s\n"
			<< "    Realtime factor:     "
			<< result.realtimeFactor << "x\n"
			<< "    Step count:          "
			<< result.stepCount << '\n'
			<< "    Deadline misses:     "
			<< result.deadlineMissCount << '\n'
			<< "    Maximum overrun:     "
			<< result.maximumOverrunS << " s\n";

		if (result.hasStabilityMetrics)
		{
			std::cout
				<< "    First-window step:   "
				<< result.firstWindowMeanStepTimeS * 1.0e6
				<< " us\n"
				<< "    Final-window step:   "
				<< result.finalWindowMeanStepTimeS * 1.0e6
				<< " us\n"
				<< "    Timing degradation:  "
				<< result.timingDegradationFraction * 100.0
				<< "%\n";
		}
	}
}

int main()
{
	try
	{
		//Math Benchmarks
		std::cout << "========== MATH ==========\n";
		const auto mathResults = aeroforge::performance::runMathBenchmarks();
		for (const auto& result : mathResults)
		{
			printBenchmarkResult(result);
		}

		std::cout << '\n';
		const auto mathSummary = aeroforge::performance::PerformanceSummary::summarize("Math", mathResults);
		printSummary(mathSummary);
		std::cout << '\n';

		//Integrator Benchmarks
		std::cout << "========== Integrator ==========\n";
		const auto integratorResults = aeroforge::performance::runIntegratorBenchmarks();
		for (const auto& result : integratorResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto integratorSummary = aeroforge::performance::PerformanceSummary::summarize("Integrator", integratorResults);
		printSummary(integratorSummary);
		std::cout << '\n';

		//SixDoF Benchmarks
		std::cout << "========== SixDoF ==========\n";
		const auto sixDoFResults = aeroforge::performance::runSixDoFBenchmarks();
		for (const auto& result : sixDoFResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto sixDoFSummary = aeroforge::performance::PerformanceSummary::summarize("SixDoF", sixDoFResults);
		printSummary(sixDoFSummary);
		std::cout << '\n';

		//GNC Benchmarks
		std::cout << "========== GNC ==========\n";
		const auto gncResults = aeroforge::performance::runGncBenchmarks();
		for (const auto& result : gncResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto gncSummary = aeroforge::performance::PerformanceSummary::summarize("GNC", gncResults);
		printSummary(gncSummary);
		std::cout << '\n';

		//Telemetry Benchmarks
		std::cout << "========== Telemetry ==========\n";
		const auto telemetryResults = aeroforge::performance::runTelemetryBenchmarks();
		for (const auto& result : telemetryResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto telemetrySummary = aeroforge::performance::PerformanceSummary::summarize("Telemetry", telemetryResults);
		printSummary(telemetrySummary);
		std::cout << '\n';

		//Monte Carlo Benchmarks
		std::cout << "========== Monte Carlo ==========\n";
		const auto monteCarloResults = aeroforge::performance::runMonteCarloBenchmarks();
		for (const auto& result : monteCarloResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto monteCarloSummary = aeroforge::performance::PerformanceSummary::summarize("Monte Carlo", monteCarloResults);
		printSummary(monteCarloSummary);
		std::cout << '\n';

		//Realtime Benchmarks
		std::cout << "========== Realtime ==========\n";
		const auto realtimeSuite = aeroforge::performance::runRealtimeBenchmarks();
		const auto& realtimeResults = realtimeSuite.benchmarks;
		for (const auto& result : realtimeResults)
		{
			printBenchmarkResult(result);
		}
		std::cout << '\n';

		std::cout << "\nRealtime / RTF metrics\n";

		for (const auto& result : realtimeSuite.realtimeMetrics)
		{
			printRealtimeBenchmarkResult(result);
		}
		std::cout << '\n';

		const auto realtimeSummary = aeroforge::performance::PerformanceSummary::summarize("Realtime", realtimeResults);
		printSummary(realtimeSummary);
		std::cout << '\n';

		//Combined
		std::cout << "========== AeroForge Overall ==========\n";
		std::vector<aeroforge::performance::BenchmarkResult> allResults;

		allResults.reserve(
			mathResults.size() +
			integratorResults.size() +
			sixDoFResults.size() +
			gncResults.size() +
			telemetryResults.size() +
			monteCarloResults.size() +
			realtimeResults.size());

		allResults.insert(
			allResults.end(),
			mathResults.begin(),
			mathResults.end());

		allResults.insert(
			allResults.end(),
			integratorResults.begin(),
			integratorResults.end());

		allResults.insert(
			allResults.end(),
			sixDoFResults.begin(),
			sixDoFResults.end());

		allResults.insert(
			allResults.end(),
			gncResults.begin(),
			gncResults.end());

		allResults.insert(
			allResults.end(),
			telemetryResults.begin(),
			telemetryResults.end());

		allResults.insert(
			allResults.end(),
			monteCarloResults.begin(),
			monteCarloResults.end());

		allResults.insert(
			allResults.end(),
			realtimeResults.begin(),
			realtimeResults.end());

		const auto allSummary = aeroforge::performance::PerformanceSummary::summarize("AeroForge Overall", allResults);
		printSummary(allSummary);
	}
	catch (const std::exception& exception)
	{
		std::cerr << "Benchmark failed: " << exception.what() << '\n';

		return 1;
	}

	return 0;
}