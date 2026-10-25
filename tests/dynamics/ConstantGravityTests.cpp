#include <gtest/gtest.h>

#include <dynamics/ConstantGravity.h>
#include <dynamics/MassProperties.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		ConstantGravityTests,
		UsesStandardGravityByDefault)
	{
		const ConstantGravity gravity;

		EXPECT_DOUBLE_EQ(
			gravity.getGravityMagnitude(),
			9.80665);
	}

	TEST(
		ConstantGravityTests,
		AcceptsCustomGravityMagnitude)
	{
		const ConstantGravity gravity(1.62);

		EXPECT_DOUBLE_EQ(
			gravity.getGravityMagnitude(),
			1.62);
	}

	TEST(
		ConstantGravityTests,
		ComputesCorrectGravityForce)
	{
		const ConstantGravity gravity;
		const MassProperties mass(10.0);

		const math::Vector3 force =
			gravity.computeForce(mass);

		EXPECT_DOUBLE_EQ(force.x, 0.0);
		EXPECT_DOUBLE_EQ(force.y, 0.0);

		EXPECT_NEAR(
			force.z,
			98.0665,
			1e-12);
	}

	TEST(
		ConstantGravityTests,
		GravityForcePointsPositiveDownInNED)
	{
		const ConstantGravity gravity;
		const MassProperties mass(5.0);

		const math::Vector3 force =
			gravity.computeForce(mass);

		EXPECT_DOUBLE_EQ(force.x, 0.0);
		EXPECT_DOUBLE_EQ(force.y, 0.0);
		EXPECT_GT(force.z, 0.0);
	}

	TEST(
		ConstantGravityTests,
		GravityForceScalesWithMass)
	{
		const ConstantGravity gravity;

		const MassProperties smallMass(10.0);
		const MassProperties largeMass(20.0);

		const math::Vector3 smallForce =
			gravity.computeForce(smallMass);

		const math::Vector3 largeForce =
			gravity.computeForce(largeMass);

		EXPECT_NEAR(
			largeForce.z,
			2.0 * smallForce.z,
			1e-12);
	}

	TEST(
		ConstantGravityTests,
		CustomGravityProducesExpectedForce)
	{
		const ConstantGravity gravity(3.5);
		const MassProperties mass(4.0);

		const math::Vector3 force =
			gravity.computeForce(mass);

		EXPECT_DOUBLE_EQ(force.x, 0.0);
		EXPECT_DOUBLE_EQ(force.y, 0.0);
		EXPECT_DOUBLE_EQ(force.z, 14.0);
	}

	TEST(
		ConstantGravityTests,
		RejectsZeroGravityMagnitude)
	{
		EXPECT_THROW(
			ConstantGravity{ 0.0 },
			std::invalid_argument);
	}

	TEST(
		ConstantGravityTests,
		RejectsNegativeGravityMagnitude)
	{
		EXPECT_THROW(
			ConstantGravity{ -9.80665 },
			std::invalid_argument);
	}

	TEST(
		ConstantGravityTests,
		RejectsNaNGravityMagnitude)
	{
		EXPECT_THROW(
			ConstantGravity{
				std::numeric_limits<double>::quiet_NaN()
			},
			std::invalid_argument);
	}

	TEST(
		ConstantGravityTests,
		RejectsPositiveInfinityGravityMagnitude)
	{
		EXPECT_THROW(
			ConstantGravity{
				std::numeric_limits<double>::infinity()
			},
			std::invalid_argument);
	}

	TEST(
		ConstantGravityTests,
		RejectsNegativeInfinityGravityMagnitude)
	{
		EXPECT_THROW(
			ConstantGravity{
				-std::numeric_limits<double>::infinity()
			},
			std::invalid_argument);
	}

	TEST(
		ConstantGravityTests,
		RejectsNonFiniteComputedGravityForce)
	{
		const ConstantGravity gravity(
			std::numeric_limits<double>::max());

		const MassProperties mass(
			std::numeric_limits<double>::max());

		EXPECT_THROW(
			gravity.computeForce(mass),
			std::runtime_error);
	}

	TEST(
		ConstantGravityTests,
		UsesUpdatedMassProperties)
	{
		const ConstantGravity gravity;
		MassProperties mass(10.0);

		const math::Vector3 first =
			gravity.computeForce(mass);

		mass.setMassKg(20.0);

		const math::Vector3 second =
			gravity.computeForce(mass);

		EXPECT_NEAR(
			first.z,
			98.0665,
			1e-12);

		EXPECT_NEAR(
			second.z,
			196.133,
			1e-12);
	}

	TEST(
		ConstantGravityTests,
		GravityAccelerationRemainsIndependentOfMass)
	{
		const ConstantGravity gravity;

		const MassProperties massA(5.0);
		const MassProperties massB(100.0);

		const math::Vector3 forceA =
			gravity.computeForce(massA);

		const math::Vector3 forceB =
			gravity.computeForce(massB);

		const double accelerationA =
			forceA.z / massA.getMassKg();

		const double accelerationB =
			forceB.z / massB.getMassKg();

		EXPECT_NEAR(
			accelerationA,
			9.80665,
			1e-12);

		EXPECT_NEAR(
			accelerationB,
			9.80665,
			1e-12);
	}
}