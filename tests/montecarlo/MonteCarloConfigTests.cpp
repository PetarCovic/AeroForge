#include <gtest/gtest.h>

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/ParameterDistribution.h>

#include <stdexcept>

namespace aeroforge::montecarlo
{
	TEST(MonteCarloConfigTests, StoresTrialCountAndSeed)
	{
		MonteCarloConfig config(
			1000,
			12345
		);

		EXPECT_EQ(
			config.getTrialCount(),
			1000
		);

		EXPECT_EQ(
			config.getBaseSeed(),
			12345u
		);
	}


	TEST(MonteCarloConfigTests, RejectsZeroTrials)
	{
		EXPECT_THROW(
			MonteCarloConfig(0, 12345),
			std::invalid_argument
		);
	}


	TEST(MonteCarloConfigTests, AddsParameterDistribution)
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

		const auto& parameters =
			config.getParameterDistributions();

		EXPECT_EQ(
			parameters.size(),
			1u
		);

		EXPECT_TRUE(
			parameters.contains("massKg")
		);
	}


	TEST(MonteCarloConfigTests, SupportsMultipleParameters)
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

		EXPECT_EQ(
			config.getParameterDistributions().size(),
			3u
		);
	}


	TEST(MonteCarloConfigTests, RejectsEmptyParameterName)
	{
		MonteCarloConfig config(
			100,
			12345
		);

		EXPECT_THROW(
			config.addParameter(
				"",
				ParameterDistribution::constant(1.0)
			),
			std::invalid_argument
		);
	}


	TEST(MonteCarloConfigTests, RejectsDuplicateParameterName)
	{
		MonteCarloConfig config(
			100,
			12345
		);

		config.addParameter(
			"massKg",
			ParameterDistribution::constant(
				100.0
			)
		);

		EXPECT_THROW(
			config.addParameter(
				"massKg",
				ParameterDistribution::constant(
					120.0
				)
			),
			std::invalid_argument
		);
	}
}