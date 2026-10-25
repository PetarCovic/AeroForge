#include <gtest/gtest.h>

#include <reproducibility/SeedManager.h>

#include <montecarlo/ParameterDistribution.h>

#include <map>
#include <random>

namespace aeroforge::reproducibility
{
	namespace
	{
		double sampleTrial(
			const SeedManager& manager,
			std::size_t trialIndex)
		{
			std::uint32_t seed =
				manager.deriveSeed(
					trialIndex,
					10
				);

			std::mt19937 engine(seed);

			auto distribution =
				montecarlo::ParameterDistribution::normal(
					100.0,
					5.0
				);

			return distribution.sample(engine);
		}
	}


	TEST(
		TrialIndependenceTests,
		TrialOutputDoesNotDependOnOtherTrialsBeingGenerated
	)
	{
		SeedManager manager(12345);

		double isolated =
			sampleTrial(
				manager,
				100
			);

		for (std::size_t i = 0;
			i < 1000;
			++i)
		{
			if (i == 100)
			{
				continue;
			}

			sampleTrial(
				manager,
				i
			);
		}

		double afterOtherTrials =
			sampleTrial(
				manager,
				100
			);

		EXPECT_DOUBLE_EQ(
			isolated,
			afterOtherTrials
		);
	}


	TEST(
		TrialIndependenceTests,
		GenerationOrderDoesNotChangeTrialValues
	)
	{
		SeedManager manager(999);

		std::map<std::size_t, double> forward;
		std::map<std::size_t, double> reverse;

		for (std::size_t i = 0;
			i < 200;
			++i)
		{
			forward[i] =
				sampleTrial(
					manager,
					i
				);
		}

		for (std::size_t i = 200;
			i > 0;
			--i)
		{
			std::size_t trial =
				i - 1;

			reverse[trial] =
				sampleTrial(
					manager,
					trial
				);
		}

		EXPECT_EQ(
			forward,
			reverse
		);
	}


	TEST(
		TrialIndependenceTests,
		DifferentStreamsWithinSameTrialAreIndependent
	)
	{
		SeedManager manager(12345);

		std::uint32_t firstSeed =
			manager.deriveSeed(
				50,
				1
			);

		std::uint32_t secondSeed =
			manager.deriveSeed(
				50,
				2
			);

		EXPECT_NE(
			firstSeed,
			secondSeed
		);

		std::mt19937 firstEngine(
			firstSeed
		);

		std::mt19937 secondEngine(
			secondSeed
		);

		auto distribution =
			montecarlo::ParameterDistribution::uniform(
				0.0,
				1.0
			);

		bool differenceFound = false;

		for (int i = 0; i < 10; ++i)
		{
			if (distribution.sample(firstEngine)
				!= distribution.sample(secondEngine))
			{
				differenceFound = true;
				break;
			}
		}

		EXPECT_TRUE(differenceFound);
	}
}