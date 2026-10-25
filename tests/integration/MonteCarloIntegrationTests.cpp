#include <gtest/gtest.h>

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/MonteCarloRunner.h>
#include <montecarlo/MonteCarloStatistics.h>
#include <montecarlo/MonteCarloTypes.h>
#include <montecarlo/ParameterDistribution.h>

#include <cmath>

namespace aeroforge::montecarlo
{
	TEST(
		MonteCarloIntegrationTests,
		RunsParameterSweepAndAggregatesResults
	)
	{
		MonteCarloConfig config(
			1000,
			12345
		);

		config.addParameter(
			"massScale",
			ParameterDistribution::uniform(
				0.9,
				1.1
			)
		);

		config.addParameter(
			"thrustScale",
			ParameterDistribution::uniform(
				0.95,
				1.05
			)
		);

		MonteCarloRunner runner(config);

		auto results = runner.run(
			[](const MonteCarloTrial& trial)
			{
				double massScale =
					trial.parameters.at(
						"massScale"
					);

				double thrustScale =
					trial.parameters.at(
						"thrustScale"
					);

				// Simple deterministic stand-in for a
				// full vehicle simulation.
				double performanceMetric =
					1000.0
					* thrustScale
					/ massScale;

				bool success =
					performanceMetric > 850.0;

				return MonteCarloResult{
					trial.trialIndex,
					success,
					{
						{
							"performance",
							performanceMetric
						}
					}
				};
			}
		);

		ASSERT_EQ(
			results.size(),
			1000u
		);

		MetricSummary summary =
			MonteCarloStatistics::summarizeMetric(
				results,
				"performance"
			);

		EXPECT_EQ(
			summary.sampleCount,
			1000u
		);

		EXPECT_TRUE(
			std::isfinite(summary.mean)
		);

		EXPECT_TRUE(
			std::isfinite(
				summary.standardDeviation
			)
		);

		EXPECT_GT(
			summary.standardDeviation,
			0.0
		);

		EXPECT_LT(
			summary.min,
			summary.max
		);

		double successRate =
			MonteCarloStatistics::computeSuccessRate(
				results
			);

		EXPECT_GE(
			successRate,
			0.0
		);

		EXPECT_LE(
			successRate,
			1.0
		);
	}


	TEST(
		MonteCarloIntegrationTests,
		CanModelAerospaceParameterUncertainty
	)
	{
		MonteCarloConfig config(
			500,
			7777
		);

		config.addParameter(
			"dryMassKg",
			ParameterDistribution::uniform(
				45.0,
				55.0
			)
		);

		config.addParameter(
			"thrustN",
			ParameterDistribution::normal(
				1000.0,
				25.0
			)
		);

		config.addParameter(
			"dragScale",
			ParameterDistribution::uniform(
				0.9,
				1.1
			)
		);

		MonteCarloRunner runner(config);

		auto results = runner.run(
			[](const MonteCarloTrial& trial)
			{
				double mass =
					trial.parameters.at(
						"dryMassKg"
					);

				double thrust =
					trial.parameters.at(
						"thrustN"
					);

				double dragScale =
					trial.parameters.at(
						"dragScale"
					);

				double acceleration =
					(thrust / mass)
					/ dragScale;

				bool success =
					acceleration > 15.0;

				return MonteCarloResult{
					trial.trialIndex,
					success,
					{
						{
							"acceleration",
							acceleration
						}
					}
				};
			}
		);

		MetricSummary summary =
			MonteCarloStatistics::summarizeMetric(
				results,
				"acceleration"
			);

		EXPECT_EQ(
			summary.sampleCount,
			500u
		);

		EXPECT_GT(
			summary.mean,
			0.0
		);

		EXPECT_GT(
			summary.standardDeviation,
			0.0
		);

		EXPECT_TRUE(
			std::isfinite(summary.mean)
		);

		EXPECT_TRUE(
			std::isfinite(summary.min)
		);

		EXPECT_TRUE(
			std::isfinite(summary.max)
		);
	}
}