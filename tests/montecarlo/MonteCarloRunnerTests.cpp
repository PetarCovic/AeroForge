#include <gtest/gtest.h>

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/MonteCarloRunner.h>
#include <montecarlo/MonteCarloTypes.h>
#include <montecarlo/ParameterDistribution.h>

#include <functional>
#include <stdexcept>
#include <vector>

namespace aeroforge::montecarlo
{
	TEST(MonteCarloRunnerTests, RunsConfiguredNumberOfTrials)
	{
		MonteCarloConfig config(
			25,
			1000
		);

		MonteCarloRunner runner(config);

		auto results = runner.run(
			[](const MonteCarloTrial& trial)
			{
				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		EXPECT_EQ(
			results.size(),
			25u
		);
	}


	TEST(MonteCarloRunnerTests, AssignsSequentialTrialIndices)
	{
		MonteCarloConfig config(
			10,
			1000
		);

		MonteCarloRunner runner(config);

		auto results = runner.run(
			[](const MonteCarloTrial& trial)
			{
				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		for (std::size_t i = 0; i < results.size(); ++i)
		{
			EXPECT_EQ(
				results[i].trialIndex,
				i
			);
		}
	}


	TEST(MonteCarloRunnerTests, AssignsSeedFromBaseSeedAndTrialIndex)
	{
		MonteCarloConfig config(
			4,
			500
		);

		MonteCarloRunner runner(config);

		std::vector<std::uint32_t> observedSeeds;

		runner.run(
			[&observedSeeds](
				const MonteCarloTrial& trial)
			{
				observedSeeds.push_back(
					trial.seed
				);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		ASSERT_EQ(
			observedSeeds.size(),
			4u
		);

		EXPECT_EQ(observedSeeds[0], 500u);
		EXPECT_EQ(observedSeeds[1], 501u);
		EXPECT_EQ(observedSeeds[2], 502u);
		EXPECT_EQ(observedSeeds[3], 503u);
	}


	TEST(MonteCarloRunnerTests, SamplesConfiguredParameters)
	{
		MonteCarloConfig config(
			100,
			12345
		);

		config.addParameter(
			"massKg",
			ParameterDistribution::uniform(
				90.0,
				110.0
			)
		);

		MonteCarloRunner runner(config);

		runner.run(
			[](const MonteCarloTrial& trial)
			{
				auto it =
					trial.parameters.find("massKg");

				EXPECT_NE(
					it,
					trial.parameters.end()
				);

				if (it != trial.parameters.end())
				{
					double massKg =
						it->second;

					EXPECT_GE(
						massKg,
						90.0
					);

					EXPECT_LE(
						massKg,
						110.0
					);
				}

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);
	}


	TEST(MonteCarloRunnerTests, SamplesEveryConfiguredParameter)
	{
		MonteCarloConfig config(
			5,
			12345
		);

		config.addParameter(
			"massKg",
			ParameterDistribution::uniform(
				90.0,
				110.0
			)
		);

		config.addParameter(
			"thrustScale",
			ParameterDistribution::normal(
				1.0,
				0.05
			)
		);

		config.addParameter(
			"dragScale",
			ParameterDistribution::constant(
				1.0
			)
		);

		MonteCarloRunner runner(config);

		runner.run(
			[](const MonteCarloTrial& trial)
			{
				EXPECT_EQ(
					trial.parameters.size(),
					3u
				);

				EXPECT_TRUE(
					trial.parameters.contains(
						"massKg"
					)
				);

				EXPECT_TRUE(
					trial.parameters.contains(
						"thrustScale"
					)
				);

				EXPECT_TRUE(
					trial.parameters.contains(
						"dragScale"
					)
				);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);
	}


	TEST(MonteCarloRunnerTests, ConstantParameterRemainsConstantAcrossTrials)
	{
		MonteCarloConfig config(
			50,
			12345
		);

		config.addParameter(
			"gravity",
			ParameterDistribution::constant(
				9.80665
			)
		);

		MonteCarloRunner runner(config);

		runner.run(
			[](const MonteCarloTrial& trial)
			{
				EXPECT_DOUBLE_EQ(
					trial.parameters.at(
						"gravity"
					),
					9.80665
				);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);
	}


	TEST(MonteCarloRunnerTests, SameConfigProducesSameSampledParameters)
	{
		MonteCarloConfig config(
			20,
			999
		);

		config.addParameter(
			"massKg",
			ParameterDistribution::uniform(
				80.0,
				120.0
			)
		);

		config.addParameter(
			"thrustScale",
			ParameterDistribution::normal(
				1.0,
				0.1
			)
		);

		MonteCarloRunner firstRunner(config);
		MonteCarloRunner secondRunner(config);

		std::vector<MonteCarloTrial> firstTrials;
		std::vector<MonteCarloTrial> secondTrials;

		firstRunner.run(
			[&firstTrials](
				const MonteCarloTrial& trial)
			{
				firstTrials.push_back(trial);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		secondRunner.run(
			[&secondTrials](
				const MonteCarloTrial& trial)
			{
				secondTrials.push_back(trial);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		ASSERT_EQ(
			firstTrials.size(),
			secondTrials.size()
		);

		for (std::size_t i = 0;
			i < firstTrials.size();
			++i)
		{
			EXPECT_EQ(
				firstTrials[i].seed,
				secondTrials[i].seed
			);

			EXPECT_EQ(
				firstTrials[i].parameters,
				secondTrials[i].parameters
			);
		}
	}


	TEST(MonteCarloRunnerTests, DifferentBaseSeedChangesSamples)
	{
		MonteCarloConfig firstConfig(
			20,
			100
		);

		MonteCarloConfig secondConfig(
			20,
			200
		);

		firstConfig.addParameter(
			"value",
			ParameterDistribution::uniform(
				0.0,
				1.0
			)
		);

		secondConfig.addParameter(
			"value",
			ParameterDistribution::uniform(
				0.0,
				1.0
			)
		);

		MonteCarloRunner firstRunner(
			firstConfig
		);

		MonteCarloRunner secondRunner(
			secondConfig
		);

		std::vector<double> firstValues;
		std::vector<double> secondValues;

		firstRunner.run(
			[&firstValues](
				const MonteCarloTrial& trial)
			{
				firstValues.push_back(
					trial.parameters.at(
						"value"
					)
				);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		secondRunner.run(
			[&secondValues](
				const MonteCarloTrial& trial)
			{
				secondValues.push_back(
					trial.parameters.at(
						"value"
					)
				);

				return MonteCarloResult{
					trial.trialIndex,
					true,
					{}
				};
			}
		);

		bool foundDifference = false;

		for (std::size_t i = 0;
			i < firstValues.size();
			++i)
		{
			if (firstValues[i]
				!= secondValues[i])
			{
				foundDifference = true;
				break;
			}
		}

		EXPECT_TRUE(foundDifference);
	}


	TEST(MonteCarloRunnerTests, PreservesCallbackMetrics)
	{
		MonteCarloConfig config(
			5,
			1000
		);

		MonteCarloRunner runner(config);

		auto results = runner.run(
			[](const MonteCarloTrial& trial)
			{
				return MonteCarloResult{
					trial.trialIndex,
					true,
					{
						{
							"score",
							static_cast<double>(
								trial.trialIndex
							)
						}
					}
				};
			}
		);

		ASSERT_EQ(
			results.size(),
			5u
		);

		EXPECT_DOUBLE_EQ(
			results[3].metrics.at("score"),
			3.0
		);
	}


	TEST(MonteCarloRunnerTests, RejectsEmptyTrialFunction)
	{
		MonteCarloConfig config(
			10,
			1000
		);

		MonteCarloRunner runner(config);

		MonteCarloRunner::TrialFunction function;

		EXPECT_THROW(
			runner.run(function),
			std::invalid_argument
		);
	}
}