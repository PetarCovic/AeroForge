#include <gtest/gtest.h>

#include <dynamics/InertiaProperties.h>

#include <math/Matrix.h>
#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		InertiaPropertiesTests,
		AcceptsPositiveDiagonalTensor)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, 3.0, 0.0,
				0.0, 0.0, 4.0
			});

		EXPECT_NO_THROW(
			InertiaProperties{
				inertia
			});
	}

	TEST(
		InertiaPropertiesTests,
		StoresInertiaTensor)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, 3.0, 0.0,
				0.0, 0.0, 4.0
			});

		const InertiaProperties properties(
			inertia);

		const math::Vector3 result =
			properties
			.getInertiaTensorBody()
			.multVector(
				math::Vector3(
					1.0,
					1.0,
					1.0));

		EXPECT_DOUBLE_EQ(result.x, 2.0);
		EXPECT_DOUBLE_EQ(result.y, 3.0);
		EXPECT_DOUBLE_EQ(result.z, 4.0);
	}

	TEST(
		InertiaPropertiesTests,
		StoresCorrectInverseTensor)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, 4.0, 0.0,
				0.0, 0.0, 5.0
			});

		const InertiaProperties properties(
			inertia);

		const math::Vector3 result =
			properties
			.getInverseInertiaTensorBody()
			.multVector(
				math::Vector3(
					2.0,
					4.0,
					5.0));

		EXPECT_NEAR(result.x, 1.0, 1e-12);
		EXPECT_NEAR(result.y, 1.0, 1e-12);
		EXPECT_NEAR(result.z, 1.0, 1e-12);
	}

	TEST(
		InertiaPropertiesTests,
		AcceptsSymmetricCoupledTensor)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				4.0, 0.2, 0.1,
				0.2, 5.0, 0.3,
				0.1, 0.3, 6.0
			});

		EXPECT_NO_THROW(
			InertiaProperties{
				inertia
			});
	}

	TEST(
		InertiaPropertiesTests,
		RejectsNonThreeByThreeMatrix)
	{
		const math::Matrix inertia(
			2,
			2,
			{
				1.0, 0.0,
				0.0, 1.0
			});

		EXPECT_THROW(
			InertiaProperties{
				inertia
			},
			std::invalid_argument);
	}

	TEST(
		InertiaPropertiesTests,
		RejectsNonSymmetricTensor)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 1.0, 0.0,
				0.0, 3.0, 0.0,
				0.0, 0.0, 4.0
			});

		EXPECT_THROW(
			InertiaProperties{
				inertia
			},
			std::invalid_argument);
	}

	TEST(
		InertiaPropertiesTests,
		RejectsZeroPrincipalInertia)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				0.0, 0.0, 4.0
			});

		EXPECT_THROW(
			InertiaProperties{
				inertia
			},
			std::invalid_argument);
	}

	TEST(
		InertiaPropertiesTests,
		RejectsNegativePrincipalInertia)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, -3.0, 0.0,
				0.0, 0.0, 4.0
			});

		EXPECT_THROW(
			InertiaProperties{
				inertia
			},
			std::invalid_argument);
	}

	TEST(
		InertiaPropertiesTests,
		RejectsNonFiniteTensor)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const math::Matrix inertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, nan, 0.0,
				0.0, 0.0, 4.0
			});

		EXPECT_THROW(
			InertiaProperties{
				inertia
			},
			std::invalid_argument);
	}
}