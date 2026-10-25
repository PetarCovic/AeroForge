#include <gtest/gtest.h>

#include <dynamics/RigidBodyQuaternionNormalization.h>

#include <simulation/State.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		RigidBodyQuaternionNormalizationTests,
		NormalizesQuaternionComponents)
	{
		simulation::State state{
			1.0, 2.0, 3.0,
			4.0, 5.0, 6.0,
			2.0, 0.0, 0.0, 0.0,
			7.0, 8.0, 9.0
		};

		normalizeRigidBodyQuaternion(
			state);

		EXPECT_DOUBLE_EQ(state[6], 1.0);
		EXPECT_DOUBLE_EQ(state[7], 0.0);
		EXPECT_DOUBLE_EQ(state[8], 0.0);
		EXPECT_DOUBLE_EQ(state[9], 0.0);
	}

	TEST(
		RigidBodyQuaternionNormalizationTests,
		PreservesNonQuaternionState)
	{
		simulation::State state{
			1.0, 2.0, 3.0,
			4.0, 5.0, 6.0,
			2.0, 0.0, 0.0, 0.0,
			7.0, 8.0, 9.0
		};

		normalizeRigidBodyQuaternion(
			state);

		EXPECT_DOUBLE_EQ(state[0], 1.0);
		EXPECT_DOUBLE_EQ(state[1], 2.0);
		EXPECT_DOUBLE_EQ(state[2], 3.0);

		EXPECT_DOUBLE_EQ(state[3], 4.0);
		EXPECT_DOUBLE_EQ(state[4], 5.0);
		EXPECT_DOUBLE_EQ(state[5], 6.0);

		EXPECT_DOUBLE_EQ(state[10], 7.0);
		EXPECT_DOUBLE_EQ(state[11], 8.0);
		EXPECT_DOUBLE_EQ(state[12], 9.0);
	}

	TEST(
		RigidBodyQuaternionNormalizationTests,
		ProducesUnitQuaternion)
	{
		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			1.0, 2.0, 3.0, 4.0,
			0.0, 0.0, 0.0
		};

		normalizeRigidBodyQuaternion(
			state);

		const double norm =
			std::sqrt(
				state[6] * state[6]
				+ state[7] * state[7]
				+ state[8] * state[8]
				+ state[9] * state[9]);

		EXPECT_NEAR(
			norm,
			1.0,
			1e-12);
	}

	TEST(
		RigidBodyQuaternionNormalizationTests,
		RejectsIncorrectStateSize)
	{
		simulation::State state{
			0.0,
			0.0,
			0.0
		};

		EXPECT_THROW(
			normalizeRigidBodyQuaternion(
				state),
			std::invalid_argument);
	}

	TEST(
		RigidBodyQuaternionNormalizationTests,
		RejectsZeroQuaternion)
	{
		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		EXPECT_THROW(
			normalizeRigidBodyQuaternion(
				state),
			std::invalid_argument);
	}

	TEST(
		RigidBodyQuaternionNormalizationTests,
		RejectsNonFiniteQuaternion)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			nan, 0.0, 0.0, 1.0,
			0.0, 0.0, 0.0
		};

		EXPECT_THROW(
			normalizeRigidBodyQuaternion(
				state),
			std::invalid_argument);
	}
}