#include <gtest/gtest.h>

#include <dynamics/RigidBodyState.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/State.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		RigidBodyStateTests,
		StateSizeIsThirteen)
	{
		EXPECT_EQ(
			RigidBodyState::STATE_SIZE,
			13);
	}

	TEST(
		RigidBodyStateTests,
		StoresRigidBodyQuantities)
	{
		const RigidBodyState state(
			math::Vector3(
				1.0,
				2.0,
				3.0),
			math::Vector3(
				4.0,
				5.0,
				6.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.1,
				0.2,
				0.3));

		EXPECT_DOUBLE_EQ(
			state.getPositionN().x,
			1.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocityN().y,
			5.0);

		EXPECT_DOUBLE_EQ(
			state.getAttitudeNB().w,
			1.0);

		EXPECT_DOUBLE_EQ(
			state.getAngularVelocityB().z,
			0.3);
	}

	TEST(
		RigidBodyStateTests,
		ConvertsToExpectedStateVectorLayout)
	{
		const RigidBodyState rigidBodyState(
			math::Vector3(
				1.0,
				2.0,
				3.0),
			math::Vector3(
				4.0,
				5.0,
				6.0),
			math::Quaternion(
				0.7,
				0.1,
				0.2,
				0.3),
			math::Vector3(
				7.0,
				8.0,
				9.0));

		const simulation::State state =
			rigidBodyState.toStateVector();

		ASSERT_EQ(state.size(), 13);

		EXPECT_DOUBLE_EQ(state[0], 1.0);
		EXPECT_DOUBLE_EQ(state[1], 2.0);
		EXPECT_DOUBLE_EQ(state[2], 3.0);

		EXPECT_DOUBLE_EQ(state[3], 4.0);
		EXPECT_DOUBLE_EQ(state[4], 5.0);
		EXPECT_DOUBLE_EQ(state[5], 6.0);

		EXPECT_DOUBLE_EQ(state[6], 0.7);
		EXPECT_DOUBLE_EQ(state[7], 0.1);
		EXPECT_DOUBLE_EQ(state[8], 0.2);
		EXPECT_DOUBLE_EQ(state[9], 0.3);

		EXPECT_DOUBLE_EQ(state[10], 7.0);
		EXPECT_DOUBLE_EQ(state[11], 8.0);
		EXPECT_DOUBLE_EQ(state[12], 9.0);
	}

	TEST(
		RigidBodyStateTests,
		RoundTripPreservesState)
	{
		const RigidBodyState original(
			math::Vector3(
				10.0,
				20.0,
				30.0),
			math::Vector3(
				40.0,
				50.0,
				60.0),
			math::Quaternion(
				0.9,
				0.1,
				0.2,
				0.3),
			math::Vector3(
				0.4,
				0.5,
				0.6));

		const simulation::State packed =
			original.toStateVector();

		const RigidBodyState reconstructed(
			packed);

		const simulation::State roundTrip =
			reconstructed.toStateVector();

		EXPECT_TRUE(
			packed.isApprox(
				roundTrip,
				1e-12));
	}

	TEST(
		RigidBodyStateTests,
		RejectsIncorrectStateSize)
	{
		const simulation::State state{
			0.0,
			0.0,
			0.0
		};

		EXPECT_THROW(
			RigidBodyState{ state },
			std::invalid_argument);
	}

	TEST(
		RigidBodyStateTests,
		RejectsNonFiniteState)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0,
			1.0, 0.0, 0.0, 0.0,
			nan, 0.0, 0.0
		};

		EXPECT_THROW(
			RigidBodyState{ state },
			std::invalid_argument);
	}

	TEST(
		RigidBodyStateTests,
		RejectsZeroQuaternion)
	{
		EXPECT_THROW(
			(RigidBodyState{
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					0.0,
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0)
				}),
			std::invalid_argument);
	}

	TEST(
		RigidBodyStateTests,
		AllowsNonUnitButNonzeroQuaternion)
	{
		EXPECT_NO_THROW(
			(RigidBodyState{
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					2.0,
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0)
				}));
	}
}