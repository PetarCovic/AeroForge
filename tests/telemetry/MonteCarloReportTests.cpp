#include <gtest/gtest.h>

#include <telemetry/MonteCarloReport.h>

#include <montecarlo/MonteCarloTypes.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace aeroforge::telemetry
{
	TEST(MonteCarloReportTests, RejectsEmptyResults)
	{
		EXPECT_THROW(
			MonteCarloReport::build(
				{},
				{ "apogeeM" }
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloReportTests, RejectsEmptyMetricName)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{
				0,
				true,
				{
					{"apogeeM", 1000.0}
				}
			}
		};

		EXPECT_THROW(
			MonteCarloReport::build(
				results,
				{ "" }
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloReportTests, ReportsTrialCount)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{0, true, {}},
			{1, false, {}},
			{2, true, {}}
		};

		auto report =
			MonteCarloReport::build(
				results,
				{}
			);

		EXPECT_EQ(
			report.trialCount,
			3u
		);
	}


	TEST(MonteCarloReportTests, ComputesSuccessRate)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{0, true, {}},
			{1, false, {}},
			{2, true, {}},
			{3, true, {}}
		};

		auto report =
			MonteCarloReport::build(
				results,
				{}
			);

		EXPECT_DOUBLE_EQ(
			report.successRate,
			0.75
		);
	}


	TEST(MonteCarloReportTests, SummarizesRequestedMetric)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{
				0,
				true,
				{{"apogeeM", 900.0}}
			},
			{
				1,
				true,
				{{"apogeeM", 1000.0}}
			},
			{
				2,
				true,
				{{"apogeeM", 1100.0}}
			}
		};

		auto report =
			MonteCarloReport::build(
				results,
				{ "apogeeM" }
			);

		ASSERT_TRUE(
			report.metricSummaries.contains(
				"apogeeM"
			)
		);

		const auto& summary =
			report.metricSummaries.at(
				"apogeeM"
			);

		EXPECT_EQ(
			summary.sampleCount,
			3u
		);

		EXPECT_DOUBLE_EQ(
			summary.mean,
			1000.0
		);

		EXPECT_DOUBLE_EQ(
			summary.min,
			900.0
		);

		EXPECT_DOUBLE_EQ(
			summary.max,
			1100.0
		);
	}


	TEST(MonteCarloReportTests, SummarizesMultipleMetrics)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{
				0,
				true,
				{
					{"apogeeM", 1000.0},
					{"maxSpeedMps", 200.0}
				}
			},

			{
				1,
				true,
				{
					{"apogeeM", 1100.0},
					{"maxSpeedMps", 220.0}
				}
			}
		};

		auto report =
			MonteCarloReport::build(
				results,
				{
					"apogeeM",
					"maxSpeedMps"
				}
			);

		EXPECT_TRUE(
			report.metricSummaries.contains(
				"apogeeM"
			)
		);

		EXPECT_TRUE(
			report.metricSummaries.contains(
				"maxSpeedMps"
			)
		);
	}


	TEST(MonteCarloReportTests, UsesExistingStatisticsMissingMetricBehavior)
	{
		std::vector<montecarlo::MonteCarloResult> results{
			{
				0,
				true,
				{{"apogeeM", 1000.0}}
			},

			{
				1,
				true,
				{}
			},

			{
				2,
				true,
				{{"apogeeM", 1200.0}}
			}
		};

		auto report =
			MonteCarloReport::build(
				results,
				{ "apogeeM" }
			);

		EXPECT_EQ(
			report.metricSummaries
			.at("apogeeM")
			.sampleCount,
			2u
		);

		EXPECT_DOUBLE_EQ(
			report.metricSummaries
			.at("apogeeM")
			.mean,
			1100.0
		);
	}
}