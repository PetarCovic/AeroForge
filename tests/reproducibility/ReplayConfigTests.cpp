#include <gtest/gtest.h>

#include <reproducibility/ReplayConfig.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace aeroforge::reproducibility
{
	TEST(ReplayConfigTests, StoresReplayConfiguration)
	{
		std::map<std::string, double> parameters{
			{"massKg", 100.0},
			{"thrustScale", 1.05}
		};

		ReplayConfig config(
			42,
			123456u,
			0.01,
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(4.0, 5.0, 6.0),
			math::Quaternion(1.0, 0.0, 0.0, 0.0),
			math::Vector3(0.1, 0.2, 0.3),
			parameters
		);

		EXPECT_EQ(
			config.getTrialIndex(),
			42u
		);

		EXPECT_EQ(
			config.getTrialSeed(),
			123456u
		);

		EXPECT_DOUBLE_EQ(
			config.getDtS(),
			0.01
		);

		EXPECT_DOUBLE_EQ(
			config.getInitialPositionN().x,
			1.0
		);

		EXPECT_DOUBLE_EQ(
			config.getInitialVelocityN().y,
			5.0
		);

		EXPECT_DOUBLE_EQ(
			config.getInitialAngularVelocityB().z,
			0.3
		);

		EXPECT_EQ(
			config.getSampledParameters(),
			parameters
		);
	}


	TEST(ReplayConfigTests, NormalizesInitialAttitude)
	{
		ReplayConfig config(
			0,
			1,
			0.01,
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(2.0, 0.0, 0.0, 0.0),
			math::Vector3(),
			{}
		);

		EXPECT_NEAR(
			config.getInitialAttitudeNB().w,
			1.0,
			1e-12
		);
	}


	TEST(ReplayConfigTests, RejectsZeroTimestep)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				0.0,
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsNegativeTimestep)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				-0.01,
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsNonFiniteTimestep)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				std::numeric_limits<double>::quiet_NaN(),
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsNonFiniteInitialPosition)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				0.01,
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0
				),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsZeroInitialAttitude)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				0.01,
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					0.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsNonFiniteSampledParameter)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				0.01,
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{
					{
						"massKg",
						std::numeric_limits<double>::quiet_NaN()
					}
				}
			),
			std::invalid_argument
		);
	}


	TEST(ReplayConfigTests, RejectsEmptySampledParameterName)
	{
		EXPECT_THROW(
			ReplayConfig(
				0,
				1,
				0.01,
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),
				{
					{"", 10.0}
				}
			),
			std::invalid_argument
		);
	}
}