#include <gtest/gtest.h>

#include <reproducibility/SeedManager.h>

#include <cstddef>
#include <cstdint>
#include <set>

namespace aeroforge::reproducibility
{
	TEST(SeedManagerTests, StoresBaseSeed)
	{
		SeedManager manager(12345);

		EXPECT_EQ(
			manager.getBaseSeed(),
			12345u
		);
	}


	TEST(SeedManagerTests, SameInputsProduceSameSeed)
	{
		SeedManager manager(12345);

		std::uint32_t first =
			manager.deriveSeed(
				42,
				1
			);

		std::uint32_t second =
			manager.deriveSeed(
				42,
				1
			);

		EXPECT_EQ(
			first,
			second
		);
	}


	TEST(SeedManagerTests, SeparateManagersWithSameBaseSeedProduceSameSeed)
	{
		SeedManager firstManager(12345);
		SeedManager secondManager(12345);

		EXPECT_EQ(
			firstManager.deriveSeed(42, 3),
			secondManager.deriveSeed(42, 3)
		);
	}


	TEST(SeedManagerTests, DifferentBaseSeedsProduceDifferentDerivedSeeds)
	{
		SeedManager firstManager(100);
		SeedManager secondManager(200);

		EXPECT_NE(
			firstManager.deriveSeed(42, 1),
			secondManager.deriveSeed(42, 1)
		);
	}


	TEST(SeedManagerTests, DifferentTrialIndicesProduceDifferentSeeds)
	{
		SeedManager manager(12345);

		std::uint32_t first =
			manager.deriveSeed(
				10,
				1
			);

		std::uint32_t second =
			manager.deriveSeed(
				11,
				1
			);

		EXPECT_NE(
			first,
			second
		);
	}


	TEST(SeedManagerTests, DifferentStreamIdsProduceDifferentSeeds)
	{
		SeedManager manager(12345);

		std::uint32_t accelerometerSeed =
			manager.deriveSeed(
				42,
				1
			);

		std::uint32_t gyroscopeSeed =
			manager.deriveSeed(
				42,
				2
			);

		EXPECT_NE(
			accelerometerSeed,
			gyroscopeSeed
		);
	}


	TEST(SeedManagerTests, SeedDerivationDoesNotDependOnCallOrder)
	{
		SeedManager firstManager(12345);

		std::uint32_t firstAccel =
			firstManager.deriveSeed(
				20,
				1
			);

		std::uint32_t firstGyro =
			firstManager.deriveSeed(
				20,
				2
			);

		std::uint32_t firstGps =
			firstManager.deriveSeed(
				20,
				3
			);


		SeedManager secondManager(12345);

		std::uint32_t secondGps =
			secondManager.deriveSeed(
				20,
				3
			);

		std::uint32_t secondAccel =
			secondManager.deriveSeed(
				20,
				1
			);

		std::uint32_t secondGyro =
			secondManager.deriveSeed(
				20,
				2
			);


		EXPECT_EQ(
			firstAccel,
			secondAccel
		);

		EXPECT_EQ(
			firstGyro,
			secondGyro
		);

		EXPECT_EQ(
			firstGps,
			secondGps
		);
	}


	TEST(SeedManagerTests, RepeatedDerivationDoesNotAdvanceInternalState)
	{
		SeedManager manager(12345);

		std::uint32_t original =
			manager.deriveSeed(
				100,
				5
			);

		for (std::uint32_t streamId = 0;
			streamId < 100;
			++streamId)
		{
			manager.deriveSeed(
				streamId,
				streamId
			);
		}

		std::uint32_t repeated =
			manager.deriveSeed(
				100,
				5
			);

		EXPECT_EQ(
			original,
			repeated
		);
	}


	TEST(SeedManagerTests, ProducesDistinctSeedsForCommonStreamsWithinTrial)
	{
		SeedManager manager(12345);

		std::set<std::uint32_t> seeds;

		for (std::uint32_t streamId = 0;
			streamId < 16;
			++streamId)
		{
			seeds.insert(
				manager.deriveSeed(
					42,
					streamId
				)
			);
		}

		EXPECT_EQ(
			seeds.size(),
			16u
		);
	}


	TEST(SeedManagerTests, ProducesDistinctSeedsAcrossCommonTrialRange)
	{
		SeedManager manager(12345);

		std::set<std::uint32_t> seeds;

		for (std::size_t trialIndex = 0;
			trialIndex < 1000;
			++trialIndex)
		{
			seeds.insert(
				manager.deriveSeed(
					trialIndex,
					1
				)
			);
		}

		EXPECT_EQ(
			seeds.size(),
			1000u
		);
	}


	TEST(SeedManagerTests, SupportsTrialIndexZeroAndStreamIdZero)
	{
		SeedManager manager(0);

		std::uint32_t seed =
			manager.deriveSeed(
				0,
				0
			);

		// There is no requirement that a derived seed itself
		// be non-zero. We are only verifying that zero-valued
		// inputs are valid and deterministic.
		EXPECT_EQ(
			seed,
			manager.deriveSeed(0, 0)
		);
	}
}