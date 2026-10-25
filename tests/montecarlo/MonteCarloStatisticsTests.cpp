#include <gtest/gtest.h>

#include <montecarlo/MonteCarloStatistics.h>
#include <montecarlo/MonteCarloTypes.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace aeroforge::montecarlo
{
	TEST(MonteCarloStatisticsTests, ComputesMetricSummary)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {{"apogeeM", 1.0}}},
			{1, true, {{"apogeeM", 2.0}}},
			{2, true, {{"apogeeM", 3.0}}}
		};

		MetricSummary summary =
			MonteCarloStatistics::summarizeMetric(
				results,
				"apogeeM"
			);

		EXPECT_EQ(
			summary.sampleCount,
			3u
		);

		EXPECT_DOUBLE_EQ(
			summary.mean,
			2.0
		);

		EXPECT_NEAR(
			summary.standardDeviation,
			std::sqrt(2.0 / 3.0),
			1e-12
		);

		EXPECT_DOUBLE_EQ(
			summary.min,
			1.0
		);

		EXPECT_DOUBLE_EQ(
			summary.max,
			3.0
		);
	}


	TEST(MonteCarloStatisticsTests, ComputesZeroStandardDeviationForConstantMetric)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {{"value", 5.0}}},
			{1, true, {{"value", 5.0}}},
			{2, true, {{"value", 5.0}}}
		};

		MetricSummary summary =
			MonteCarloStatistics::summarizeMetric(
				results,
				"value"
			);

		EXPECT_DOUBLE_EQ(
			summary.mean,
			5.0
		);

		EXPECT_DOUBLE_EQ(
			summary.standardDeviation,
			0.0
		);
	}


	TEST(MonteCarloStatisticsTests, SkipsResultsWithoutRequestedMetric)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {{"apogeeM", 100.0}}},
			{1, false, {}},
			{2, true, {{"apogeeM", 200.0}}}
		};

		MetricSummary summary =
			MonteCarloStatistics::summarizeMetric(
				results,
				"apogeeM"
			);

		EXPECT_EQ(
			summary.sampleCount,
			2u
		);

		EXPECT_DOUBLE_EQ(
			summary.mean,
			150.0
		);
	}


	TEST(MonteCarloStatisticsTests, ComputesSuccessRate)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {}},
			{1, false, {}},
			{2, true, {}},
			{3, true, {}}
		};

		EXPECT_DOUBLE_EQ(
			MonteCarloStatistics::computeSuccessRate(
				results
			),
			0.75
		);
	}


	TEST(MonteCarloStatisticsTests, RejectsEmptyResultSetForSuccessRate)
	{
		std::vector<MonteCarloResult> results;

		EXPECT_THROW(
			MonteCarloStatistics::computeSuccessRate(
				results
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloStatisticsTests, RejectsMetricThatDoesNotExist)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {{"apogeeM", 100.0}}}
		};

		EXPECT_THROW(
			MonteCarloStatistics::summarizeMetric(
				results,
				"impactSpeed"
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloStatisticsTests, RejectsEmptyMetricName)
	{
		std::vector<MonteCarloResult> results{
			{0, true, {{"apogeeM", 100.0}}}
		};

		EXPECT_THROW(
			MonteCarloStatistics::summarizeMetric(
				results,
				""
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloStatisticsTests, RejectsNonFiniteMetricValue)
	{
		std::vector<MonteCarloResult> results{
			{
				0,
				true,
				{
					{
						"apogeeM",
						std::numeric_limits<double>::quiet_NaN()
					}
				}
			}
		};

		EXPECT_THROW(
			MonteCarloStatistics::summarizeMetric(
				results,
				"apogeeM"
			),
			std::invalid_argument
		);
	}
}