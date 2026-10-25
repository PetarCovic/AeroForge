#include <gtest/gtest.h>

#include <telemetry/RunMetadata.h>

#include <limits>
#include <map>
#include <stdexcept>

namespace aeroforge::telemetry
{
	TEST(RunMetadataTests, StoresRunInformation)
	{
		std::map<std::string, double> parameters{
			{"massScale", 1.05},
			{"thrustScale", 0.98}
		};

		RunMetadata metadata(
			"run-001",
			12345u,
			42u,
			67890u,
			0.01,
			parameters
		);

		EXPECT_EQ(
			metadata.getRunId(),
			"run-001"
		);

		EXPECT_EQ(
			metadata.getBaseSeed(),
			12345u
		);

		EXPECT_EQ(
			metadata.getTrialIndex(),
			42u
		);

		EXPECT_EQ(
			metadata.getTrialSeed(),
			67890u
		);

		EXPECT_DOUBLE_EQ(
			metadata.getDtS(),
			0.01
		);

		EXPECT_EQ(
			metadata.getSampledParameters(),
			parameters
		);
	}


	TEST(RunMetadataTests, RejectsEmptyRunId)
	{
		EXPECT_THROW(
			RunMetadata(
				"",
				1,
				0,
				2,
				0.01,
				{}
			),
			std::invalid_argument
		);
	}


	TEST(RunMetadataTests, RejectsZeroTimestep)
	{
		EXPECT_THROW(
			RunMetadata(
				"run",
				1,
				0,
				2,
				0.0,
				{}
			),
			std::invalid_argument
		);
	}


	TEST(RunMetadataTests, RejectsNonFiniteTimestep)
	{
		EXPECT_THROW(
			RunMetadata(
				"run",
				1,
				0,
				2,
				std::numeric_limits<double>::quiet_NaN(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(RunMetadataTests, RejectsEmptyParameterName)
	{
		EXPECT_THROW(
			RunMetadata(
				"run",
				1,
				0,
				2,
				0.01,
				{
					{"", 1.0}
				}
			),
			std::invalid_argument
		);
	}


	TEST(RunMetadataTests, RejectsNonFiniteParameterValue)
	{
		EXPECT_THROW(
			RunMetadata(
				"run",
				1,
				0,
				2,
				0.01,
				{
					{
						"massScale",
						std::numeric_limits<double>::infinity()
					}
				}
			),
			std::invalid_argument
		);
	}
}