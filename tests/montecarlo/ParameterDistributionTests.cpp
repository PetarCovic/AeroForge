#include <gtest/gtest.h>

#include <montecarlo/ParameterDistribution.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace aeroforge::montecarlo
{
	TEST(ParameterDistributionTests, ConstantAlwaysReturnsConfiguredValue)
	{
		ParameterDistribution distribution =
			ParameterDistribution::constant(5.5);

		std::mt19937 engine(1234);

		for (int i = 0; i < 100; ++i)
		{
			EXPECT_DOUBLE_EQ(
				distribution.sample(engine),
				5.5
			);
		}
	}


	TEST(ParameterDistributionTests, UniformSamplesRemainWithinBounds)
	{
		ParameterDistribution distribution =
			ParameterDistribution::uniform(
				-2.0,
				4.0
			);

		std::mt19937 engine(1234);

		for (int i = 0; i < 1000; ++i)
		{
			double value =
				distribution.sample(engine);

			EXPECT_GE(value, -2.0);
			EXPECT_LE(value, 4.0);
		}
	}


	TEST(ParameterDistributionTests, UniformAllowsEqualBounds)
	{
		ParameterDistribution distribution =
			ParameterDistribution::uniform(
				3.0,
				3.0
			);

		std::mt19937 engine(1234);

		for (int i = 0; i < 10; ++i)
		{
			EXPECT_DOUBLE_EQ(
				distribution.sample(engine),
				3.0
			);
		}
	}


	TEST(ParameterDistributionTests, NormalDistributionHasApproximatelyExpectedMean)
	{
		ParameterDistribution distribution =
			ParameterDistribution::normal(
				10.0,
				2.0
			);

		std::mt19937 engine(12345);

		const int sampleCount = 50000;

		double sum = 0.0;

		for (int i = 0; i < sampleCount; ++i)
		{
			sum += distribution.sample(engine);
		}

		double mean =
			sum / sampleCount;

		EXPECT_NEAR(
			mean,
			10.0,
			0.05
		);
	}


	TEST(ParameterDistributionTests, NormalAllowsZeroStandardDeviation)
	{
		ParameterDistribution distribution =
			ParameterDistribution::normal(
				7.0,
				0.0
			);

		std::mt19937 engine(100);

		for (int i = 0; i < 10; ++i)
		{
			EXPECT_DOUBLE_EQ(
				distribution.sample(engine),
				7.0
			);
		}
	}


	TEST(ParameterDistributionTests, SameSeedProducesSameSequence)
	{
		ParameterDistribution distribution =
			ParameterDistribution::normal(
				0.0,
				1.0
			);

		std::mt19937 firstEngine(555);
		std::mt19937 secondEngine(555);

		for (int i = 0; i < 100; ++i)
		{
			EXPECT_DOUBLE_EQ(
				distribution.sample(firstEngine),
				distribution.sample(secondEngine)
			);
		}
	}


	TEST(ParameterDistributionTests, RejectsNonFiniteConstant)
	{
		EXPECT_THROW(
			ParameterDistribution::constant(
				std::numeric_limits<double>::quiet_NaN()
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			ParameterDistribution::constant(
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}


	TEST(ParameterDistributionTests, RejectsInvalidUniformDistribution)
	{
		EXPECT_THROW(
			ParameterDistribution::uniform(
				5.0,
				2.0
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			ParameterDistribution::uniform(
				std::numeric_limits<double>::quiet_NaN(),
				2.0
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			ParameterDistribution::uniform(
				0.0,
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}


	TEST(ParameterDistributionTests, RejectsInvalidNormalDistribution)
	{
		EXPECT_THROW(
			ParameterDistribution::normal(
				0.0,
				-1.0
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			ParameterDistribution::normal(
				std::numeric_limits<double>::quiet_NaN(),
				1.0
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			ParameterDistribution::normal(
				0.0,
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}
}