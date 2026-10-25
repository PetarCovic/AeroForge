#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <telemetry/CsvTelemetryWriter.h>
#include <telemetry/FlightAnalyzer.h>
#include <telemetry/GncAnalyzer.h>
#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace aeroforge::performance
{
	namespace
	{
		volatile double benchmarkSink = 0.0;

		telemetry::TelemetrySample makeSample(
			double timeS)
		{
			return telemetry::TelemetrySample{
				timeS,

				math::Vector3(
					timeS * 100.0,
					timeS * 2.0,
					-100.0 - timeS * 10.0),

				math::Vector3(
					100.0,
					2.0,
					-10.0),

				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),

				math::Vector3(
					0.01,
					-0.02,
					0.03),

				math::Vector3(
					timeS * 100.0 + 0.1,
					timeS * 2.0 - 0.1,
					-100.0 - timeS * 10.0),

				math::Vector3(
					100.1,
					1.9,
					-10.0),

				math::Quaternion(
					0.999999,
					0.0005,
					0.0005,
					0.0005)
				.normalized(),

				math::Vector3(
					0.011,
					-0.019,
					0.031),

				math::Vector3(
					0.01,
					-0.02,
					0.015),

				math::Vector3(
					1.0,
					-0.5,
					0.25),

				math::Vector3(
					0.95,
					-0.48,
					0.24),

				0.75,

				0.01,

				-0.02,

				false
			};
		}

		std::vector<telemetry::TelemetrySample>
			createSamples(
				std::size_t count)
		{
			std::vector<telemetry::TelemetrySample>
				samples;

			samples.reserve(count);

			for (std::size_t i = 0;
				i < count;
				++i)
			{
				samples.push_back(
					makeSample(
						static_cast<double>(i) *
						0.01));
			}

			return samples;
		}

		template <typename Operation>
		BenchmarkResult runBatchedBenchmark(
			const std::string& name,
			std::size_t batchCount,
			std::size_t iterationsPerBatch,
			Operation operation)
		{
			double totalTimeS = 0.0;
			double minTimeS =
				std::numeric_limits<double>::max();
			double maxTimeS = 0.0;

			for (std::size_t batch = 0;
				batch < batchCount;
				++batch)
			{
				PerformanceTimer timer;

				timer.start();

				const double sinkValue =
					operation(
						iterationsPerBatch);

				const double batchTimeS =
					timer.stop();

				benchmarkSink = sinkValue;

				const double batchMeanTimeS =
					batchTimeS /
					static_cast<double>(
						iterationsPerBatch);

				totalTimeS += batchTimeS;

				minTimeS =
					std::min(
						minTimeS,
						batchMeanTimeS);

				maxTimeS =
					std::max(
						maxTimeS,
						batchMeanTimeS);
			}

			const std::size_t totalIterations =
				batchCount *
				iterationsPerBatch;

			return BenchmarkResult{
				name,
				totalIterations,
				totalTimeS,
				totalTimeS /
					static_cast<double>(
						totalIterations),
				minTimeS,
				maxTimeS,
				totalTimeS > 0.0
					? static_cast<double>(
						totalIterations) /
						totalTimeS
					: 0.0
			};
		}

		BenchmarkResult benchmarkTelemetryRecorder()
		{
			return runBatchedBenchmark(
				"TelemetryRecorder",
				10,
				100'000,
				[](std::size_t iterations)
				{
					telemetry::TelemetryRecorder recorder;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						recorder.record(
							makeSample(
								static_cast<double>(i) *
								0.001));
					}

					return static_cast<double>(
						recorder.size());
				});
		}

		BenchmarkResult benchmarkFlightAnalyzer()
		{
			const std::vector<
				telemetry::TelemetrySample> samples =
				createSamples(10'000);

			return runBatchedBenchmark(
				"FlightAnalyzer10kSamples",
				10,
				100,
				[&samples](
					std::size_t iterations)
				{
					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const telemetry::FlightMetrics
							metrics =
							telemetry::FlightAnalyzer::
							analyze(samples);

						accumulator +=
							metrics.maxAltitudeM +
							metrics.maxSpeedMps +
							metrics.maxAccelerationMps2;
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkGncAnalyzer()
		{
			const std::vector<
				telemetry::TelemetrySample> samples =
				createSamples(10'000);

			return runBatchedBenchmark(
				"GncAnalyzer10kSamples",
				10,
				100,
				[&samples](
					std::size_t iterations)
				{
					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const telemetry::GncMetrics metrics =
							telemetry::GncAnalyzer::
							analyze(
								samples,
								0.05);

						accumulator +=
							metrics.rmsAttitudeErrorRad +
							metrics.peakAttitudeErrorRad +
							metrics.meanCommandedMomentNm;
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkCsvTelemetryWriter()
		{
			const std::vector<
				telemetry::TelemetrySample> samples =
				createSamples(10'000);

			return runBatchedBenchmark(
				"CsvTelemetryWriter10kSamples",
				10,
				10,
				[&samples](
					std::size_t iterations)
				{
					std::size_t totalCharacters = 0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						std::ostringstream output;

						telemetry::CsvTelemetryWriter::write(
							output,
							samples);

						const std::streampos position = output.tellp();

						if (position > 0)
						{
							totalCharacters +=
								static_cast<std::size_t>(position);
						}
					}

					return static_cast<double>(
						totalCharacters);
				});
		}
	}

	std::vector<BenchmarkResult>
		runTelemetryBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(4);

		results.push_back(
			benchmarkTelemetryRecorder());

		results.push_back(
			benchmarkFlightAnalyzer());

		results.push_back(
			benchmarkGncAnalyzer());

		results.push_back(
			benchmarkCsvTelemetryWriter());

		return results;
	}
}