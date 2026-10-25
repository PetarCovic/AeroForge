#include <gtest/gtest.h>

#include <dynamics/InertiaProperties.h>
#include <dynamics/RotationalDynamics.h>

#include <math/Matrix.h>
#include <math/Vector3.h>

namespace aeroforge::dynamics::tests
{
	TEST(
		RotationalDynamicsTests,
		ZeroMomentAndZeroRateProduceZeroAcceleration)
	{
		const InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}));

		const math::Vector3 acceleration =
			computeAngularAccelerationBody(
				inertia,
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0));

		EXPECT_DOUBLE_EQ(acceleration.x, 0.0);
		EXPECT_DOUBLE_EQ(acceleration.y, 0.0);
		EXPECT_DOUBLE_EQ(acceleration.z, 0.0);
	}

	TEST(
		RotationalDynamicsTests,
		AppliesMomentThroughInertia)
	{
		const InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 4.0, 0.0,
					0.0, 0.0, 5.0
				}));

		const math::Vector3 acceleration =
			computeAngularAccelerationBody(
				inertia,
				math::Vector3(
					4.0,
					8.0,
					10.0),
				math::Vector3(
					0.0,
					0.0,
					0.0));

		EXPECT_NEAR(acceleration.x, 2.0, 1e-12);
		EXPECT_NEAR(acceleration.y, 2.0, 1e-12);
		EXPECT_NEAR(acceleration.z, 2.0, 1e-12);
	}

	TEST(
		RotationalDynamicsTests,
		IncludesGyroscopicCoupling)
	{
		const InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}));

		const math::Vector3 acceleration =
			computeAngularAccelerationBody(
				inertia,
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					1.0,
					2.0,
					3.0));

		EXPECT_NEAR(
			acceleration.x,
			-3.0,
			1e-12);

		EXPECT_NEAR(
			acceleration.y,
			2.0,
			1e-12);

		EXPECT_NEAR(
			acceleration.z,
			-0.5,
			1e-12);
	}

	TEST(
		RotationalDynamicsTests,
		SphericalInertiaEliminatesGyroscopicCoupling)
	{
		const InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					5.0, 0.0, 0.0,
					0.0, 5.0, 0.0,
					0.0, 0.0, 5.0
				}));

		const math::Vector3 acceleration =
			computeAngularAccelerationBody(
				inertia,
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					1.0,
					2.0,
					3.0));

		EXPECT_NEAR(acceleration.x, 0.0, 1e-12);
		EXPECT_NEAR(acceleration.y, 0.0, 1e-12);
		EXPECT_NEAR(acceleration.z, 0.0, 1e-12);
	}
}