#include <gtest/gtest.h>

#include <reproducibility/SeedManager.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GpsModel.h>
#include <sensors/GyroscopeModel.h>

#include <math/Vector3.h>

namespace aeroforge::reproducibility
{
	TEST(
		SensorReproducibilityTests,
		AccelerometersWithSameSeedProduceExactSameSequence
	)
	{
		SeedManager seeds(12345);

		std::uint32_t seed =
			seeds.deriveSeed(42, 1);

		sensors::AccelerometerModel first(
			math::Vector3(0.1, -0.2, 0.3),
			math::Vector3(0.01, 0.02, 0.03),
			math::Vector3(100.0, 100.0, 100.0),
			seed
		);

		sensors::AccelerometerModel second(
			math::Vector3(0.1, -0.2, 0.3),
			math::Vector3(0.01, 0.02, 0.03),
			math::Vector3(100.0, 100.0, 100.0),
			seed
		);

		for (int i = 0; i < 100; ++i)
		{
			math::Vector3 input(
				1.0,
				2.0,
				3.0
			);

			math::Vector3 a =
				first.measure(input);

			math::Vector3 b =
				second.measure(input);

			EXPECT_DOUBLE_EQ(a.x, b.x);
			EXPECT_DOUBLE_EQ(a.y, b.y);
			EXPECT_DOUBLE_EQ(a.z, b.z);
		}
	}


	TEST(
		SensorReproducibilityTests,
		GyroscopesWithSameSeedProduceExactSameSequence
	)
	{
		SeedManager seeds(12345);

		std::uint32_t seed =
			seeds.deriveSeed(42, 2);

		sensors::GyroscopeModel first(
			math::Vector3(0.001, 0.002, 0.003),
			math::Vector3(0.01, 0.01, 0.01),
			math::Vector3(10.0, 10.0, 10.0),
			seed
		);

		sensors::GyroscopeModel second(
			math::Vector3(0.001, 0.002, 0.003),
			math::Vector3(0.01, 0.01, 0.01),
			math::Vector3(10.0, 10.0, 10.0),
			seed
		);

		for (int i = 0; i < 100; ++i)
		{
			math::Vector3 a =
				first.measure(
					math::Vector3(
						0.1,
						0.2,
						0.3
					)
				);

			math::Vector3 b =
				second.measure(
					math::Vector3(
						0.1,
						0.2,
						0.3
					)
				);

			EXPECT_DOUBLE_EQ(a.x, b.x);
			EXPECT_DOUBLE_EQ(a.y, b.y);
			EXPECT_DOUBLE_EQ(a.z, b.z);
		}
	}


	TEST(
		SensorReproducibilityTests,
		GpsModelsWithSameSeedProduceExactSameSequence
	)
	{
		SeedManager seeds(12345);

		std::uint32_t seed =
			seeds.deriveSeed(42, 3);

		sensors::GpsModel first(
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(0.5, 0.5, 0.5),
			math::Vector3(0.1, 0.2, 0.3),
			math::Vector3(0.05, 0.05, 0.05),
			seed
		);

		sensors::GpsModel second(
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(0.5, 0.5, 0.5),
			math::Vector3(0.1, 0.2, 0.3),
			math::Vector3(0.05, 0.05, 0.05),
			seed
		);

		for (int i = 0; i < 100; ++i)
		{
			double timeS =
				i * 0.1;

			auto a = first.sample(
				timeS,
				math::Vector3(10.0, 20.0, 30.0),
				math::Vector3(1.0, 2.0, 3.0)
			);

			auto b = second.sample(
				timeS,
				math::Vector3(10.0, 20.0, 30.0),
				math::Vector3(1.0, 2.0, 3.0)
			);

			EXPECT_DOUBLE_EQ(a.timeS, b.timeS);

			EXPECT_DOUBLE_EQ(
				a.positionN.x,
				b.positionN.x
			);

			EXPECT_DOUBLE_EQ(
				a.positionN.y,
				b.positionN.y
			);

			EXPECT_DOUBLE_EQ(
				a.positionN.z,
				b.positionN.z
			);

			EXPECT_DOUBLE_EQ(
				a.velocityN.x,
				b.velocityN.x
			);

			EXPECT_DOUBLE_EQ(
				a.velocityN.y,
				b.velocityN.y
			);

			EXPECT_DOUBLE_EQ(
				a.velocityN.z,
				b.velocityN.z
			);
		}
	}


	TEST(
		SensorReproducibilityTests,
		DifferentAccelerometerSeedsChangeNoiseSequence
	)
	{
		SeedManager manager(12345);

		sensors::AccelerometerModel first(
			math::Vector3(),
			math::Vector3(1.0, 1.0, 1.0),
			math::Vector3(100.0, 100.0, 100.0),
			manager.deriveSeed(1, 1)
		);

		sensors::AccelerometerModel second(
			math::Vector3(),
			math::Vector3(1.0, 1.0, 1.0),
			math::Vector3(100.0, 100.0, 100.0),
			manager.deriveSeed(2, 1)
		);

		bool foundDifference = false;

		for (int i = 0; i < 20; ++i)
		{
			auto a =
				first.measure(math::Vector3());

			auto b =
				second.measure(math::Vector3());

			if (a.x != b.x
				|| a.y != b.y
				|| a.z != b.z)
			{
				foundDifference = true;
				break;
			}
		}

		EXPECT_TRUE(foundDifference);
	}
}