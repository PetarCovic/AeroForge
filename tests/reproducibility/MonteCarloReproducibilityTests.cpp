#include <gtest/gtest.h>

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/MonteCarloRunner.h>
#include <montecarlo/ParameterDistribution.h>

namespace aeroforge::reproducibility
{
	TEST(
		MonteCarloReproducibilityTests,
		EntireBatchReplaysExactly
	)
	{
		montecarlo::MonteCarloConfig config(
			250,
			12345
		);

		config.addParameter(
			"massKg",
			montecarlo::ParameterDistribution::uniform(
				90.0,
				110.0
			)
		);

		config.addParameter(
			"thrustScale",
			montecarlo::ParameterDistribution::normal(
				1.0,
				0.05
			)
		);

		montecarlo::MonteCarloRunner firstRunner(
			config
		);

		montecarlo::MonteCarloRunner secondRunner(
			config
		);

		auto trialFunction =
			[](
				const montecarlo::MonteCarloTrial& trial)
			{
				double mass =
					trial.parameters.at(
						"massKg"
					);

				double thrust =
					trial.parameters.at(
						"thrustScale"
					);

				double metric =
					thrust / mass;

				return montecarlo::MonteCarloResult{
					trial.trialIndex,
					metric > 0.008,
					{
						{"metric", metric}
					}
				};
			};


		auto first =
			firstRunner.run(trialFunction);

		auto second =
			secondRunner.run(trialFunction);

		ASSERT_EQ(
			first.size(),
			second.size()
		);

		for (std::size_t i = 0;
			i < first.size();
			++i)
		{
			EXPECT_EQ(
				first[i].trialIndex,
				second[i].trialIndex
			);

			EXPECT_EQ(
				first[i].success,
				second[i].success
			);

			EXPECT_EQ(
				first[i].metrics,
				second[i].metrics
			);
		}
	}


	TEST(
		MonteCarloReproducibilityTests,
		DifferentBaseSeedChangesBatch
	)
	{
		montecarlo::MonteCarloConfig firstConfig(
			100,
			1000
		);

		montecarlo::MonteCarloConfig secondConfig(
			100,
			2000
		);

		firstConfig.addParameter(
			"x",
			montecarlo::ParameterDistribution::uniform(
				0.0,
				1.0
			)
		);

		secondConfig.addParameter(
			"x",
			montecarlo::ParameterDistribution::uniform(
				0.0,
				1.0
			)
		);

		montecarlo::MonteCarloRunner firstRunner(
			firstConfig
		);

		montecarlo::MonteCarloRunner secondRunner(
			secondConfig
		);

		auto callback =
			[](
				const montecarlo::MonteCarloTrial& trial)
			{
				return montecarlo::MonteCarloResult{
					trial.trialIndex,
					true,
					{
						{
							"x",
							trial.parameters.at("x")
						}
					}
				};
			};

		auto first =
			firstRunner.run(callback);

		auto second =
			secondRunner.run(callback);

		bool differenceFound = false;

		for (std::size_t i = 0;
			i < first.size();
			++i)
		{
			if (first[i].metrics.at("x")
				!= second[i].metrics.at("x"))
			{
				differenceFound = true;
				break;
			}
		}

		EXPECT_TRUE(differenceFound);
	}
}